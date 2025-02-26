#![allow(unknown_lints)]
#![allow(unused_variables)]
#![allow(dead_code)]
#![allow(clippy::suboptimal_flops)]
#![allow(clippy::similar_names)]
#![allow(clippy::many_single_char_names)]
#![allow(clippy::multiple_crate_versions)]

use crate::position::Position;

mod constants;
mod nav;
mod position;
mod visibility;

fn main() {
    let p1 = Position::new(1.0, 2.0, 3.0);
    let p2 = Position::new(4.0, 5.0, 6.0);

    println!("p1: {p1:?}");
    println!("p2: {p2:?}");
    println!("p1 + p2: {:?}", p1 + p2);
    println!("Can jump: {}", p1.can_jump_to(&p2));
}
