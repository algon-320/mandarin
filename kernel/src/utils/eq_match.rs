/// Generates a match expression with maching arms which matches to the given values.
///
/// # Examples
/// ```
/// let x = 10;
/// let y = 3;
/// let z = 5;
/// let result =
///     eq_match! { x;
///         y => 1,
///         7 => 2,
///         z * 2 => 3,
///         ; otherwise => { println!("x = {}", otherwise); 4 }
///     };
/// assert_eq!(result, 3);
/// ```
/// This example is equivalent to the following if-else chain:
/// ```
/// let x = 10;
/// let y = 3;
/// let z = 5;
/// let result =
///     if x == y { 1 }
///     else if x == 7 { 2 }
///     else if x == z * 2 { 3 }
///     else { let otherwise = x; println!("x = {}", otherwise); 4 };
/// assert_eq!(result, 3);
/// ```
///
/// # Notes
/// The maching arm shadows a variable named `_eq_match_temp`.
#[macro_export]
macro_rules! eq_match {
    ($e:expr ; $( $value:expr => $body:expr ),* $(,)? ; $( $ow:pat $(if $ow_guard:expr)? => $ow_body:expr ),* $(,)?)
    => {
        match $e {
            $( _eq_match_temp if _eq_match_temp == $value => $body ),* ,
            $( $ow $( if $ow_guard )? => $ow_body ),*
        }
    };
}
