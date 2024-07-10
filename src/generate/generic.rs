use core::marker;

/// Conversions to and from peripheral struts to memory blocks.
pub trait FromBytes: Sized {
    /// Copies bytes into a new peripheral struct.
    ///
    /// None will be returned if the number of bytes is not correct.
    ///
    /// # Safety
    ///
    /// The bytes must be a valid representation of the peripheral struct and the registers.
    unsafe fn from_bytes(bytes: &[u8]) -> Option<Self> {
        let self_size = std::mem::size_of::<Self>();
        if bytes.len() != self_size {
            return None;
        }
        let mut self_uninit = std::mem::MaybeUninit::<Self>::uninit();
        let self_ptr = self_uninit.as_mut_ptr();
        let bytes_ptr = bytes.as_ptr();

        // SAFETY: `MaybeUninit` ensures size and alignment are correct
        unsafe { std::ptr::copy(bytes_ptr, self_ptr as _, self_size) };

        // SAFETY: Safety note in function doc ensures called gives
        // bytes that represent valid initialization.
        Some(unsafe { self_uninit.assume_init() })
    }

    /// Converts a peripheral struct into a byte slice.
    fn as_bytes_ref(&self) -> &[u8] {
        unsafe {
            core::slice::from_raw_parts(
                (self as *const Self) as *const u8,
                core::mem::size_of::<Self>(),
            )
        }
    }
}

/// Raw register type (`u8`, `u16`, `u32`, ...)
pub trait RawReg:
    Copy
    + Default
    + From<bool>
    + core::ops::BitOr<Output = Self>
    + core::ops::BitAnd<Output = Self>
    + core::ops::BitOrAssign
    + core::ops::BitAndAssign
    + core::ops::Not<Output = Self>
    + core::ops::Shl<u8, Output = Self>
{
    /// Mask for bits of width `WI`
    fn mask<const WI: u8>() -> Self;
    /// Mask for bits of width 1
    fn one() -> Self;
}

macro_rules! raw_reg {
    ($U:ty, $size:literal, $mask:ident) => {
        impl RawReg for $U {
            #[inline(always)]
            fn mask<const WI: u8>() -> Self {
                $mask::<WI>()
            }
            #[inline(always)]
            fn one() -> Self {
                1
            }
        }
        const fn $mask<const WI: u8>() -> $U {
            <$U>::MAX >> ($size - WI)
        }
        impl FieldSpec for $U {
            type Ux = $U;
        }
    };
}

raw_reg!(u8, 8, mask_u8);
raw_reg!(u16, 16, mask_u16);
raw_reg!(u32, 32, mask_u32);
raw_reg!(u64, 64, mask_u64);

/// Raw register type
pub trait RegisterSpec {
    /// Raw register type (`u8`, `u16`, `u32`, ...).
    type Ux: RawReg;
    /// Offset into peripheral in bytes.
    const OFFSET: u64;
}

/// Raw field type
pub trait FieldSpec: Sized {
    /// Raw field type (`u8`, `u16`, `u32`, ...).
    type Ux: Copy + PartialEq + From<Self>;
}

/// Trait implemented by readable registers to enable the `read` method.
///
/// Registers marked with `Writable` can be also be `modify`'ed.
pub trait Readable: RegisterSpec {}

/// Trait implemented by writeable registers.
///
/// This enables the  `write`, `write_with_zero` and `reset` methods.
///
/// Registers marked with `Readable` can be also be `modify`'ed.
pub trait Writable: RegisterSpec {
    /// Is it safe to write any bits to register
    type Safety;

    /// Specifies the register bits that are not changed if you pass `1` and are changed if you pass `0`
    const ZERO_TO_MODIFY_FIELDS_BITMAP: Self::Ux;

    /// Specifies the register bits that are not changed if you pass `0` and are changed if you pass `1`
    const ONE_TO_MODIFY_FIELDS_BITMAP: Self::Ux;
}

/// Reset value of the register.
///
/// This value is the initial value for the `write` method. It can also be directly written to the
/// register by using the `reset` method.
pub trait Resettable: RegisterSpec {
    /// Reset value of the register.
    const RESET_VALUE: Self::Ux;

    /// Reset value of the register.
    #[inline(always)]
    fn reset_value() -> Self::Ux {
        Self::RESET_VALUE
    }
}

/// This structure provides volatile access to registers.
#[repr(transparent)]
pub struct Reg<REG: RegisterSpec> {
    register: vcell::VolatileCell<REG::Ux>,
    _marker: marker::PhantomData<REG>,
}

unsafe impl<REG: RegisterSpec> Send for Reg<REG> where REG::Ux: Send {}

impl<REG: RegisterSpec> Reg<REG> {
    /// Returns the underlying memory address of register.
    ///
    /// ```ignore
    /// let reg_ptr = periph.reg.as_ptr();
    /// ```
    #[inline(always)]
    pub fn as_ptr(&self) -> *mut REG::Ux {
        self.register.as_ptr()
    }

    /// Offset into peripheral in bytes.
    #[inline(always)]
    pub const fn offset() -> u64 {
        REG::OFFSET
    }

    /// Reads the contents of a register.
    ///
    /// # Safety
    ///
    /// This register operation occurs from the perspective of the
    /// hardware (allows reads to WO registers) and as such is `unsafe`.
    ///
    /// You can read the raw contents of a register by using `bits`:
    /// ```ignore
    /// let bits = periph.reg.sys_read().bits();
    /// ```
    /// or get the content of a particular field of a register:
    /// ```ignore
    /// let reader = periph.reg.sys_read();
    /// let bits = reader.field1().bits();
    /// let flag = reader.field2().bit_is_set();
    /// ```
    #[inline(always)]
    pub unsafe fn sys_read(&self) -> R<REG> {
        R {
            bits: self.register.get(),
            _reg: marker::PhantomData,
        }
    }

    /// Writes bits to a register.
    ///
    /// # Safety
    ///
    /// This register operation occurs from the perspective of the
    /// hardware (allows writes to RO registers) and as such is `unsafe`.
    ///
    /// # NOTE
    ///
    /// Fields not set will be written as `0`.
    ///
    /// You can write raw bits into a register:
    /// ```ignore
    /// periph.reg.sys_write_with_zero(|w| unsafe { w.bits(rawbits) });
    /// ```
    /// or write only the fields you need:
    /// ```ignore
    /// periph.reg.sys_write_with_zero(|w| w
    ///     .field1().bits(newfield1bits)
    ///     .field2().set_bit()
    ///     .field3().variant(VARIANT)
    /// );
    /// ```
    /// or an alternative way of saying the same:
    /// ```ignore
    /// periph.reg.sys_write_with_zero(|w| {
    ///     w.field1().bits(newfield1bits);
    ///     w.field2().set_bit();
    ///     w.field3().variant(VARIANT)
    /// });
    /// ```
    #[inline(always)]
    pub unsafe fn sys_write_with_zero<F>(&self, f: F)
    where
        F: FnOnce(&mut W<REG>) -> &mut W<REG>,
    {
        self.register.set(
            f(&mut W {
                bits: REG::Ux::default(),
                _reg: marker::PhantomData,
            })
            .bits,
        );
    }
    /// Modifies the contents of the register by reading and then writing it.
    ///
    /// # Safety
    ///
    /// This register operation occurs from the perspective of the
    /// hardware (allows reads/writes to WO/RO registers)
    /// and as such is `unsafe`.
    ///
    /// E.g. to do a read-modify-write sequence to change parts of a register:
    /// ```ignore
    /// periph.reg.sys_modify(|r, w| unsafe { w.bits(
    ///    r.bits() | 3
    /// ) });
    /// ```
    /// or
    /// ```ignore
    /// periph.reg.sys_modify(|_, w| w
    ///     .field1().bits(newfield1bits)
    ///     .field2().set_bit()
    ///     .field3().variant(VARIANT)
    /// );
    /// ```
    /// or an alternative way of saying the same:
    /// ```ignore
    /// periph.reg.sys_modify(|_, w| {
    ///     w.field1().bits(newfield1bits);
    ///     w.field2().set_bit();
    ///     w.field3().variant(VARIANT)
    /// });
    /// ```
    /// Other fields will have the value they had before the call to `sys_modify`.
    #[inline(always)]
    pub unsafe fn sys_modify<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&R<REG>, &'w mut W<REG>) -> &'w mut W<REG>,
    {
        let bits = self.register.get();
        self.register.set(
            f(
                &R {
                    bits,
                    _reg: marker::PhantomData,
                },
                &mut W {
                    bits,
                    _reg: marker::PhantomData,
                },
            )
            .bits,
        );
    }

    /// Clears the register content to 0
    ///
    /// # Safety
    ///
    /// This does not obey R/W permissions and is unsafe.
    ///
    /// This can be used as an alternative to `sys_reset` when
    /// it is not available.
    #[inline(always)]
    pub unsafe fn sys_register_clear(&self) {
        self.register.set(REG::Ux::default())
    }
}

impl<REG: Resettable> Reg<REG> {
    /// Resets the register content to the reset value
    ///
    /// # Safety
    ///
    /// This does not obey R/W permissions and is unsafe.
    ///
    /// Note that not all registers implement [`Resettable`],
    /// so you may have to use `sys_register_clear()` instead.
    #[inline(always)]
    pub unsafe fn sys_reset(&self) {
        self.register.set(REG::RESET_VALUE)
    }
}

impl<REG: Readable> Reg<REG> {
    /// Reads the contents of a `Readable` register.
    ///
    /// You can read the raw contents of a register by using `bits`:
    /// ```ignore
    /// let bits = periph.reg.read().bits();
    /// ```
    /// or get the content of a particular field of a register:
    /// ```ignore
    /// let reader = periph.reg.read();
    /// let bits = reader.field1().bits();
    /// let flag = reader.field2().bit_is_set();
    /// ```
    #[inline(always)]
    pub fn read(&self) -> R<REG> {
        R {
            bits: self.register.get(),
            _reg: marker::PhantomData,
        }
    }
}

impl<REG: Resettable + Writable> Reg<REG> {
    /// Writes the reset value to `Writable` register.
    ///
    /// Resets the register to its initial state.
    #[inline(always)]
    pub fn reset(&self) {
        self.register.set(REG::RESET_VALUE)
    }

    /// Writes bits to a `Writable` register.
    ///
    /// You can write raw bits into a register:
    /// ```ignore
    /// periph.reg.write(|w| unsafe { w.bits(rawbits) });
    /// ```
    /// or write only the fields you need:
    /// ```ignore
    /// periph.reg.write(|w| w
    ///     .field1().bits(newfield1bits)
    ///     .field2().set_bit()
    ///     .field3().variant(VARIANT)
    /// );
    /// ```
    /// or an alternative way of saying the same:
    /// ```ignore
    /// periph.reg.write(|w| {
    ///     w.field1().bits(newfield1bits);
    ///     w.field2().set_bit();
    ///     w.field3().variant(VARIANT)
    /// });
    /// ```
    /// In the latter case, other fields will be set to their reset value.
    #[inline(always)]
    pub fn write<F>(&self, f: F)
    where
        F: FnOnce(&mut W<REG>) -> &mut W<REG>,
    {
        self.register.set(
            f(&mut W {
                bits: REG::RESET_VALUE & !REG::ONE_TO_MODIFY_FIELDS_BITMAP
                    | REG::ZERO_TO_MODIFY_FIELDS_BITMAP,
                _reg: marker::PhantomData,
            })
            .bits,
        );
    }
}

impl<REG: Writable> Reg<REG> {
    /// Writes 0 to a `Writable` register.
    ///
    /// Similar to `write`, but unused bits will contain 0.
    ///
    /// # Safety
    ///
    /// Unsafe to use with registers which don't allow to write 0.
    #[inline(always)]
    pub unsafe fn write_with_zero<F>(&self, f: F)
    where
        F: FnOnce(&mut W<REG>) -> &mut W<REG>,
    {
        self.register.set(
            f(&mut W {
                bits: REG::Ux::default(),
                _reg: marker::PhantomData,
            })
            .bits,
        );
    }
}

impl<REG: Readable + Writable> Reg<REG> {
    /// Modifies the contents of the register by reading and then writing it.
    ///
    /// E.g. to do a read-modify-write sequence to change parts of a register:
    /// ```ignore
    /// periph.reg.modify(|r, w| unsafe { w.bits(
    ///    r.bits() | 3
    /// ) });
    /// ```
    /// or
    /// ```ignore
    /// periph.reg.modify(|_, w| w
    ///     .field1().bits(newfield1bits)
    ///     .field2().set_bit()
    ///     .field3().variant(VARIANT)
    /// );
    /// ```
    /// or an alternative way of saying the same:
    /// ```ignore
    /// periph.reg.modify(|_, w| {
    ///     w.field1().bits(newfield1bits);
    ///     w.field2().set_bit();
    ///     w.field3().variant(VARIANT)
    /// });
    /// ```
    /// Other fields will have the value they had before the call to `modify`.
    #[inline(always)]
    pub fn modify<F>(&self, f: F)
    where
        for<'w> F: FnOnce(&R<REG>, &'w mut W<REG>) -> &'w mut W<REG>,
    {
        let bits = self.register.get();
        self.register.set(
            f(
                &R {
                    bits,
                    _reg: marker::PhantomData,
                },
                &mut W {
                    bits: bits & !REG::ONE_TO_MODIFY_FIELDS_BITMAP
                        | REG::ZERO_TO_MODIFY_FIELDS_BITMAP,
                    _reg: marker::PhantomData,
                },
            )
            .bits,
        );
    }
}

#[doc(hidden)]
pub mod raw {
    use super::{marker, BitM, FieldSpec, RegisterSpec, Unsafe};

    pub struct R<REG: RegisterSpec> {
        pub(crate) bits: REG::Ux,
        pub(super) _reg: marker::PhantomData<REG>,
    }

    pub struct W<REG: RegisterSpec> {
        ///Writable bits
        pub(crate) bits: REG::Ux,
        pub(super) _reg: marker::PhantomData<REG>,
    }

    pub struct FieldReader<FI = u8>
    where
        FI: FieldSpec,
    {
        pub(crate) bits: FI::Ux,
        _reg: marker::PhantomData<FI>,
    }

    impl<FI: FieldSpec> FieldReader<FI> {
        /// Creates a new instance of the reader.
        #[inline(always)]
        pub(crate) const fn new(bits: FI::Ux) -> Self {
            Self {
                bits,
                _reg: marker::PhantomData,
            }
        }
    }

    pub struct BitReader<FI = bool> {
        pub(crate) bits: bool,
        _reg: marker::PhantomData<FI>,
    }

    impl<FI> BitReader<FI> {
        /// Creates a new instance of the reader.
        #[inline(always)]
        pub(crate) const fn new(bits: bool) -> Self {
            Self {
                bits,
                _reg: marker::PhantomData,
            }
        }
    }

    pub struct FieldWriter<'a, REG, const WI: u8, FI = u8, Safety = Unsafe>
    where
        REG: RegisterSpec,
        FI: FieldSpec,
    {
        pub(crate) w: &'a mut W<REG>,
        pub(crate) o: u8,
        _field: marker::PhantomData<(FI, Safety)>,
    }

    impl<'a, REG, const WI: u8, FI, Safety> FieldWriter<'a, REG, WI, FI, Safety>
    where
        REG: RegisterSpec,
        FI: FieldSpec,
    {
        /// Creates a new instance of the writer
        #[inline(always)]
        pub(crate) fn new(w: &'a mut W<REG>, o: u8) -> Self {
            Self {
                w,
                o,
                _field: marker::PhantomData,
            }
        }
    }

    pub struct BitWriter<'a, REG, FI = bool, M = BitM>
    where
        REG: RegisterSpec,
        bool: From<FI>,
    {
        pub(crate) w: &'a mut W<REG>,
        pub(crate) o: u8,
        _field: marker::PhantomData<(FI, M)>,
    }

    impl<'a, REG, FI, M> BitWriter<'a, REG, FI, M>
    where
        REG: RegisterSpec,
        bool: From<FI>,
    {
        /// Creates a new instance of the writer
        #[inline(always)]
        pub(crate) fn new(w: &'a mut W<REG>, o: u8) -> Self {
            Self {
                w,
                o,
                _field: marker::PhantomData,
            }
        }
    }
}

/// Register reader.
///
/// Result of the `read` methods of registers. Also used as a closure argument in the `modify`
/// method.
pub type R<REG> = raw::R<REG>;

impl<REG: RegisterSpec> R<REG> {
    /// Reads raw bits from the register.
    ///
    /// # Safety
    ///
    /// This is a system hardware level operation that ignores permissions.
    #[inline(always)]
    pub unsafe fn sys_get(&self) -> REG::Ux {
        self.bits
    }
}

impl<REG: RegisterSpec> R<REG> {
    /// Reads raw bits from register.
    #[inline(always)]
    pub const fn bits(&self) -> REG::Ux {
        self.bits
    }
}

impl<REG: RegisterSpec, FI> PartialEq<FI> for R<REG>
where
    REG::Ux: PartialEq,
    FI: Copy,
    REG::Ux: From<FI>,
{
    #[inline(always)]
    fn eq(&self, other: &FI) -> bool {
        self.bits.eq(&REG::Ux::from(*other))
    }
}
/// Register writer.
///
/// Used as an argument to the closures in the `write` and `modify` methods of the register.
pub type W<REG> = raw::W<REG>;

impl<REG: RegisterSpec> W<REG> {
    /// Writes raw bits to the register.
    ///
    /// # Safety
    ///
    /// This is a system hardware level operation that ignores permissions.
    #[inline(always)]
    pub unsafe fn sys_set(&mut self, bits: REG::Ux) -> &mut Self {
        self.bits = bits;
        self
    }
}

impl<REG: Writable> W<REG> {
    /// Writes raw bits to the register.
    ///
    /// # Safety
    ///
    /// Passing incorrect value can cause undefined behaviour. See reference manual
    #[inline(always)]
    pub unsafe fn bits(&mut self, bits: REG::Ux) -> &mut Self {
        self.bits = bits;
        self
    }
}
impl<REG> W<REG>
where
    REG: Writable<Safety = Safe>,
{
    /// Writes raw bits to the register.
    #[inline(always)]
    pub fn set(&mut self, bits: REG::Ux) -> &mut Self {
        self.bits = bits;
        self
    }
}

/// Field reader.
///
/// Result of the `read` methods of fields.
pub type FieldReader<FI = u8> = raw::FieldReader<FI>;

/// Bit-wise field reader
pub type BitReader<FI = bool> = raw::BitReader<FI>;

impl<FI: FieldSpec> FieldReader<FI> {
    /// Reads raw bits from field.
    #[inline(always)]
    pub const fn bits(&self) -> FI::Ux {
        self.bits
    }
}

impl<FI> PartialEq<FI> for FieldReader<FI>
where
    FI: FieldSpec + Copy,
{
    #[inline(always)]
    fn eq(&self, other: &FI) -> bool {
        self.bits.eq(&FI::Ux::from(*other))
    }
}

impl<FI> PartialEq<FI> for BitReader<FI>
where
    FI: Copy,
    bool: From<FI>,
{
    #[inline(always)]
    fn eq(&self, other: &FI) -> bool {
        self.bits.eq(&bool::from(*other))
    }
}

impl<FI> BitReader<FI> {
    /// Value of the field as raw bits.
    #[inline(always)]
    pub const fn bit(&self) -> bool {
        self.bits
    }
    /// Returns `true` if the bit is clear (0).
    #[inline(always)]
    pub const fn bit_is_clear(&self) -> bool {
        !self.bit()
    }
    /// Returns `true` if the bit is set (1).
    #[inline(always)]
    pub const fn bit_is_set(&self) -> bool {
        self.bit()
    }
}

/// Marker for register/field writers which can take any value of specified width
pub struct Safe;
/// You should check that value is allowed to pass to register/field writer marked with this
pub struct Unsafe;

/// Write field Proxy with unsafe `bits`
pub type FieldWriter<'a, REG, const WI: u8, FI = u8> = raw::FieldWriter<'a, REG, WI, FI, Unsafe>;
/// Write field Proxy with safe `bits`
pub type FieldWriterSafe<'a, REG, const WI: u8, FI = u8> = raw::FieldWriter<'a, REG, WI, FI, Safe>;

impl<'a, REG, const WI: u8, FI> FieldWriter<'a, REG, WI, FI>
where
    REG: RegisterSpec,
    FI: FieldSpec,
    REG::Ux: From<FI::Ux>,
{
    /// Field width
    pub const WIDTH: u8 = WI;

    /// Field width
    #[inline(always)]
    pub const fn width(&self) -> u8 {
        WI
    }

    /// Field offset
    #[inline(always)]
    pub const fn offset(&self) -> u8 {
        self.o
    }

    /// Writes raw bits to the field
    ///
    /// # Safety
    ///
    /// Passing incorrect value can cause undefined behaviour. See reference manual
    #[inline(always)]
    pub unsafe fn bits(self, value: FI::Ux) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::mask::<WI>() << self.o);
        self.w.bits |= (REG::Ux::from(value) & REG::Ux::mask::<WI>()) << self.o;
        self.w
    }
    /// Writes `variant` to the field
    #[inline(always)]
    pub fn variant(self, variant: FI) -> &'a mut W<REG> {
        unsafe { self.bits(FI::Ux::from(variant)) }
    }
}

impl<'a, REG, const WI: u8, FI> FieldWriterSafe<'a, REG, WI, FI>
where
    REG: RegisterSpec,
    FI: FieldSpec,
    REG::Ux: From<FI::Ux>,
{
    /// Field width
    pub const WIDTH: u8 = WI;

    /// Field width
    #[inline(always)]
    pub const fn width(&self) -> u8 {
        WI
    }

    /// Field offset
    #[inline(always)]
    pub const fn offset(&self) -> u8 {
        self.o
    }

    /// Writes raw bits to the field
    #[inline(always)]
    pub fn bits(self, value: FI::Ux) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::mask::<WI>() << self.o);
        self.w.bits |= (REG::Ux::from(value) & REG::Ux::mask::<WI>()) << self.o;
        self.w
    }
    /// Writes `variant` to the field
    #[inline(always)]
    pub fn variant(self, variant: FI) -> &'a mut W<REG> {
        self.bits(FI::Ux::from(variant))
    }
}

macro_rules! bit_proxy {
    ($writer:ident, $mwv:ident) => {
        #[doc(hidden)]
        pub struct $mwv;

        /// Bit-wise write field proxy
        pub type $writer<'a, REG, FI = bool> = raw::BitWriter<'a, REG, FI, $mwv>;

        impl<'a, REG, FI> $writer<'a, REG, FI>
        where
            REG: RegisterSpec,
            bool: From<FI>,
        {
            /// Field width
            pub const WIDTH: u8 = 1;

            /// Field width
            #[inline(always)]
            pub const fn width(&self) -> u8 {
                Self::WIDTH
            }

            /// Field offset
            #[inline(always)]
            pub const fn offset(&self) -> u8 {
                self.o
            }

            /// Writes bit to the field
            #[inline(always)]
            pub fn bit(self, value: bool) -> &'a mut W<REG> {
                self.w.bits &= !(REG::Ux::one() << self.o);
                self.w.bits |= (REG::Ux::from(value) & REG::Ux::one()) << self.o;
                self.w
            }
            /// Writes `variant` to the field
            #[inline(always)]
            pub fn variant(self, variant: FI) -> &'a mut W<REG> {
                self.bit(bool::from(variant))
            }
        }
    };
}

bit_proxy!(BitWriter, BitM);
bit_proxy!(BitWriter1S, Bit1S);
bit_proxy!(BitWriter0C, Bit0C);
bit_proxy!(BitWriter1C, Bit1C);
bit_proxy!(BitWriter0S, Bit0S);
bit_proxy!(BitWriter1T, Bit1T);
bit_proxy!(BitWriter0T, Bit0T);

impl<'a, REG, FI> BitWriter<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    /// Sets the field bit
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W<REG> {
        self.w.bits |= REG::Ux::one() << self.o;
        self.w
    }
    /// Clears the field bit
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::one() << self.o);
        self.w
    }
}

impl<'a, REG, FI> BitWriter1S<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    /// Sets the field bit
    #[inline(always)]
    pub fn set_bit(self) -> &'a mut W<REG> {
        self.w.bits |= REG::Ux::one() << self.o;
        self.w
    }
}

impl<'a, REG, FI> BitWriter0C<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    /// Clears the field bit
    #[inline(always)]
    pub fn clear_bit(self) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::one() << self.o);
        self.w
    }
}

impl<'a, REG, FI> BitWriter1C<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    ///Clears the field bit by passing one
    #[inline(always)]
    pub fn clear_bit_by_one(self) -> &'a mut W<REG> {
        self.w.bits |= REG::Ux::one() << self.o;
        self.w
    }
}

impl<'a, REG, FI> BitWriter0S<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    ///Sets the field bit by passing zero
    #[inline(always)]
    pub fn set_bit_by_zero(self) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::one() << self.o);
        self.w
    }
}

impl<'a, REG, FI> BitWriter1T<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    ///Toggle the field bit by passing one
    #[inline(always)]
    pub fn toggle_bit(self) -> &'a mut W<REG> {
        self.w.bits |= REG::Ux::one() << self.o;
        self.w
    }
}

impl<'a, REG, FI> BitWriter0T<'a, REG, FI>
where
    REG: RegisterSpec,
    bool: From<FI>,
{
    ///Toggle the field bit by passing zero
    #[inline(always)]
    pub fn toggle_bit(self) -> &'a mut W<REG> {
        self.w.bits &= !(REG::Ux::one() << self.o);
        self.w
    }
}
