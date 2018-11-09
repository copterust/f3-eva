module mpu9250 (board_w = 26.1, board_d = 15.6) {
    padding = 2.6;
    pin_w = 25.1;
    r = 1.5;
    r_ofs_y = 11.4;
    r_ofs_x = 1.6;
    board_h = 1.6;

    plate_d = board_d + padding;
    plate_w = board_w + padding;
    center_w = plate_w / 2.0;

    pin_hole = center_w - (pin_w / 2.0);
    union () {
        // Plate with cut-out for pins
        difference() {
            cube([plate_w, plate_d, 2.6]);
            translate([pin_hole, padding, -padding / 2.0])
                cube([pin_w, 2.6, 2.6 + padding]);
        }
        // Pegs
        difference() {
            union () {
                translate([r + padding / 2.0 + r_ofs_x, 11.4 + padding, 2.6]) {
                    cylinder(h = board_h, r1 = r, r2 = r, $fn = 100);
                }
                translate([r + padding / 2.0 + r_ofs_x, 11.4 + padding, 2.6 + board_h])
                    cylinder(h = board_h, r1 = r + 1.0, r2 = r * 0.9, $fn = 100);
            }
            translate([padding / 2.0 + r_ofs_x * 0.9 - 1.0, 11.4 + padding - 0.5, 2.6]) {
                cube([r * 2.2 + 2.0, 1.0, board_h + 2.6]);
            }
        }
        difference() {
            union() {
                translate([plate_w - r - padding / 2.0 - r_ofs_x, 11.4 + padding, 2.6])
                    cylinder(h = board_h, r1 = r, r2 = r, $fn = 100);
                translate([plate_w - r - padding / 2.0 - r_ofs_x, 11.4 + padding, 2.6 + board_h])
                    cylinder(h = board_h, r1 = r + 1.0, r2 = r * 0.9, $fn = 100);
            }
            translate([plate_w - padding / 2.0 - r_ofs_x - r * 2.1 - 1.0, 11.4 + padding - 0.5, 2.6]) {
                cube([r * 2.2 + 2.0, 1.0, board_h + 2.6]);
            }
        }
    }
    
}

mpu9250();