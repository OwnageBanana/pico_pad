//! each key is assigned a pin on this macropad
struct Key {
    isMacro: bool,
    code: u8,
    pin: Pin<T, Input<PullUp>>,
}


