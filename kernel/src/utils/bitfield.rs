/// Defines a method to get bit-field.
///
/// # Example
/// ```
/// struct Foo { data: u32 }
/// impl Foo {
///     getter!(data: u32; 0x0000FF00; u8, pub bar)
/// }
/// let foo = Foo { data: 0x12345678 };
/// assert_eq!(foo.bar(), 0x56);
/// ```
///
/// The getter! defines a public method named 'bar'
/// which extracts bits 8:15 of the member 'data' as u8.
#[macro_export]
macro_rules! getter {
    ($base:tt $([$idx:literal])? : $base_ty:ty ; $mask:expr ; $ty:ty, $vis:vis $getter_name:ident) => {
        $vis fn $getter_name(&self) -> $ty {
            (((self.$base $([$idx])?) & $mask) >> <$base_ty>::trailing_zeros($mask)) as $ty
        }
    };
}

/// Defines a method to set bit-field.
///
/// # Example
/// ```
/// struct Foo { data: u32 }
/// impl Foo {
///     setter!(data: u32; 0x0000FF00; u8, pub bar)
/// }
/// let mut foo = Foo { data: 0x12345678 }
/// foo.bar(0x00);
/// assert_eq!(foo.data, 0x12340078);
/// ```
///
/// The setter! defines a public method named 'bar'
/// which replaces bits 8:15 of the member 'data' with a given u8 value.
#[macro_export]
macro_rules! setter {
    ($base:tt $([$idx:literal])? : $base_ty:ty ; $mask:expr ; $ty:ty, $vis:vis $setter_name:ident) => {
        $vis fn $setter_name(&mut self, val: $ty) {
            self.$base $([$idx])? = self.$base $([$idx])? & !$mask | ((val as $base_ty) << <$base_ty>::trailing_zeros($mask));
        }
    };
}
