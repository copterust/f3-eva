module mpu9250 (board_w = 26.1, board_d = 15.6) {
    padding = 2.6;
    pin_w = 25.1 + 0.5;
    r = 1.7;
    r_ofs_y = 11.4;
    r_ofs_x = 1.0;
    board_h = 1.6;

    plate_d = board_d + padding;
    plate_w = board_w + padding;
    center_w = plate_w / 2.0;

    offset = 0.4;

    pin_hole = center_w - (pin_w / 2.0);

    // Plate with cut-out for pins
    difference() {
        cube([plate_w, plate_d, 2.6]);
        translate([pin_hole, padding - 1.4 + offset, -padding / 2.0])
            cube([pin_w, 2.6 + 0.2, 2.6 + padding]);

        translate([r + padding / 2.0 + r_ofs_x, 11.4 + padding + offset, -0.5]) {
            cylinder(h = 3.0 + 0.5, r1 = r, r2 = r, $fn = 100);
        }

        translate([plate_w - r - padding / 2.0 - r_ofs_x, 11.4 + padding + offset, -0.5])
            cylinder(h = 3.0 + 0.5, r1 = r, r2 = r, $fn = 100);
        }
}

mpu9250();