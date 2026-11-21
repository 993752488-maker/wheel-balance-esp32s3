/*
 * TCRT5000 enclosure V1 / mm / first fit prototype.
 * Measured: PCB 32 x 14.2; component height 6.6; sensor-side envelope 11.3.
 * PCB thickness and sensor footprint position are editable assumptions.
 * Mechanical parts only; fasteners, zip ties, pads and electronics not included.
 * Exports select part: body, lid, arm, saddle, plate, assembly, pcb_gauge.
 */

part = "assembly";
show_electronics = true;
$fn = 48;
eps = 0.02;

pcb_l = 32;
pcb_w = 14.2;
pcb_t = 1.6;                 // ASSUMED: change if actual thickness differs.
component_h = 6.6;
sensor_h = 11.3;             // From sensor-side PCB face to optical face.
sensor_x = 28;               // ASSUMED: opposite connector end, for fit check only.
sensor_y = 0;                // ASSUMED: centred across PCB.
sensor_axial = 5.8;
sensor_transverse = 10.2;

pcb_gap = 0.5;
component_side_clearance = 2;
wall = 2;
floor_t = 2;
optical_recess = 0.8;        // Nearest plastic sits behind optical face.
sensor_zone_start = 18;      // Generous open nose; not a sealed optical tunnel.

back_z = sensor_h;
front_z = back_z + pcb_t;
case_top = front_z + component_h + 1.5;
lid_t = 2.4;
lid_top = case_top + lid_t;
arm_t = 4;
arm_top = lid_top + arm_t;
inner_x0 = -pcb_gap;
inner_x1 = pcb_l + pcb_gap;
inner_y = pcb_w/2 + component_side_clearance;
outer_x0 = inner_x0-wall;
outer_x1 = inner_x1+wall;
outer_y = inner_y+wall;

cover_screw_x = [3,29];
cover_screw_y = [-13.5,13.5];
ear_r = 4.6;
bolt_d = 3.5;
nut_af = 5.9;
nut_well_roof = 10.8;        // M3 x 20 reaches nut but remains inside ear.
clamp_gap = 0.05;

slot_start = 52;
slot_end = 72;
slot_y = 8;
saddle_x = (slot_start+slot_end)/2;
saddle_l = 34;
saddle_w = 30;
saddle_h = 15;
saddle_v_floor = 6;
saddle_v_angle = 120;
tie_width = 6;
tie_height = 2.4;
tie_centres = [-11,11];

assert(pcb_l>0 && pcb_w>0 && pcb_t>0);
assert(sensor_zone_start < sensor_x-sensor_axial/2);
assert(nut_well_roof < back_z);
assert(abs(sensor_y)+sensor_transverse/2 < pcb_w/2-0.7);
assert(slot_start-saddle_l/2 > outer_x1);

module round_box(x0,x1,y0,y1,z0,h,r=1.2) {
    translate([x0,y0,z0]) linear_extrude(height=h)
        offset(r=r) translate([r,r]) square([x1-x0-2*r,y1-y0-2*r]);
}

module hex_well(x,y,z,h) {
    translate([x,y,z]) rotate([0,0,30])
        cylinder(d=nut_af/cos(30),h=h,$fn=6);
}

module cover_ears(z,h) {
    for(x=cover_screw_x,y=cover_screw_y)
        translate([x,y,z]) cylinder(r=ear_r,h=h);
}

module cover_holes(z,h) {
    for(x=cover_screw_x,y=cover_screw_y)
        translate([x,y,z]) cylinder(d=bolt_d,h=h);
}

module body() {
    difference() {
        union() {
            difference() {
                round_box(outer_x0,outer_x1,-outer_y,outer_y,
                          optical_recess,case_top-optical_recess);
                translate([inner_x0,-inner_y,optical_recess+floor_t])
                    cube([inner_x1-inner_x0,2*inner_y,case_top+eps]);
                // Open nose: optical face and probe cannot be covered by a window.
                translate([sensor_zone_start,-(pcb_w/2+0.5),-eps])
                    cube([outer_x1-sensor_zone_start+2,pcb_w+1,case_top+2]);
                // Connector opening, with room above the bent pin header.
                translate([outer_x0-1,-(pcb_w/2+0.8),back_z-0.5])
                    cube([4,pcb_w+1.6,case_top-back_z+2]);
            }
            cover_ears(optical_recess,case_top-optical_recess);
            // PCB rests on the long edges, not on components or the optical head.
            for(s=[-1,1]) {
                translate([0.6,s==1?pcb_w/2-0.7:-inner_y-0.2,optical_recess])
                    cube([pcb_l-1.2,inner_y+0.2-(pcb_w/2-0.7),back_z-optical_recess]);
                translate([0.6,s==1?pcb_w/2+pcb_gap:-(pcb_w/2+pcb_gap+0.8),back_z-0.4])
                    cube([pcb_l-1.2,0.8,pcb_t+1.6]);
                // Cable tie-through posts: a tie can hold the plug/cable loom.
                round_box(-8,0.5,s==1?10.2:-16,s==1?16:-10.2,
                          optical_recess,14.2,0.9);
                // End stops at the PCB corners, clear of the central optical bay.
                translate([pcb_l+0.35,s==1?pcb_w/2-0.7:-inner_y,
                           back_z-0.8])
                    cube([outer_x1-(pcb_l+0.35),inner_y-(pcb_w/2-0.7),pcb_t+2.2]);
                // 45-degree underside ramp avoids a horizontal unsupported tab.
                hull() {
                    translate([pcb_l+0.35,s==1?pcb_w/2+0.5:-inner_y,back_z-2.0])
                        cube([outer_x1-(pcb_l+0.35),inner_y-(pcb_w/2+0.5),0.03]);
                    translate([pcb_l+0.35,s==1?pcb_w/2-0.7:-inner_y,back_z-0.8])
                        cube([outer_x1-(pcb_l+0.35),inner_y-(pcb_w/2-0.7),0.03]);
                }
            }
        }
        cover_holes(-eps,case_top+2);
        // Hex access from underside. Nut is drawn up to the internal shoulder.
        for(x=cover_screw_x,y=cover_screw_y)
            hex_well(x,y,-eps,nut_well_roof+eps);
        for(s=[-1,1])
            translate([-6.5,s==1?12:-14.2,-eps])
                cube([5.5,2.2,16]);
    }
}

module lid() {
    difference() {
        union() {
            round_box(outer_x0,outer_x1,-outer_y,outer_y,case_top,lid_t);
            cover_ears(case_top,lid_t);
            for(x=[3.5,23],s=[-1,1]) {
                translate([x,s==1?pcb_w/2-0.65:-(pcb_w/2+0.35),front_z+clamp_gap])
                    cube([3,1,14.4-front_z-clamp_gap]);
                translate([x,s==1?pcb_w/2-0.65:-(inner_y-0.5),14.3])
                    cube([3,inner_y-0.5-(pcb_w/2-0.65),case_top-14.3+0.15]);
            }
            // Short locator lips keep the cap from moving sideways.
            for(s=[-1,1])
                translate([7,s==1?inner_y-0.45:-inner_y+0.15,case_top-1.2])
                    cube([18,0.3,1.4]);
        }
        cover_holes(case_top-1,lid_t+3);
        // Elongated access window tolerates unmeasured potentiometer/LED positions.
        round_box(3.5,27.5,-5.2,5.2,case_top-1,lid_t+2,1.4);
        translate([outer_x0-0.1,-(pcb_w/2+0.8),case_top-0.1])
            cube([4.6,pcb_w+1.6,lid_t+0.2]);
    }
}

module slot(x0,x1,y,z,h,d=bolt_d) {
    hull() for(x=[x0,x1]) translate([x,y,z]) cylinder(d=d,h=h);
}

module arm() {
    difference() {
        union() {
            round_box(outer_x0,outer_x1,-outer_y,outer_y,lid_top,arm_t);
            cover_ears(lid_top,arm_t);
            round_box(27,slot_end+6,-13,13,lid_top,arm_t,2);
        }
        cover_holes(lid_top-1,arm_t+2);
        // Keep the lid access window unobstructed by the mounting arm.
        round_box(2.5,28.5,-7.2,7.2,lid_top-1,arm_t+2,1.8);
        for(y=[-slot_y,slot_y])slot(slot_start,slot_end,y,lid_top-1,arm_t+2);
        translate([outer_x0-0.1,-(pcb_w/2+0.8),lid_top-0.1])
            cube([4.6,pcb_w+1.6,arm_t+0.2]);
    }
}

module saddle() {
    difference() {
        // Extrude the V section along the tube axis (X). No support roof needed.
        rotate([90,0,90]) linear_extrude(height=saddle_l,center=true)
            polygon([[-saddle_w/2,0],[saddle_w/2,0],
                     [saddle_w/2,saddle_v_floor+saddle_w/2*tan((180-saddle_v_angle)/2)],
                     [0,saddle_v_floor],
                     [-saddle_w/2,saddle_v_floor+saddle_w/2*tan((180-saddle_v_angle)/2)]]);
        for(x=tie_centres)
            translate([x-tie_width/2,-saddle_w/2-1,0.9])
                cube([tie_width,saddle_w+2,tie_height]);
        for(y=[-slot_y,slot_y]) {
            translate([0,y,-eps])cylinder(d=bolt_d,h=saddle_h+2);
            hex_well(0,y,-eps,2.8+eps);
        }
    }
}

module pcb_proxy() {
    translate([0,-pcb_w/2,back_z])cube([pcb_l,pcb_w,pcb_t]);
}

module sensor_proxy() {
    translate([sensor_x-sensor_axial/2,sensor_y-sensor_transverse/2,0])
        cube([sensor_axial,sensor_transverse,sensor_h]);
}

module body_print() {translate([0,0,-optical_recess])body();}
module lid_print() {translate([0,0,lid_top])rotate([180,0,0])lid();}
module arm_print() {translate([0,0,-lid_top])arm();}
module saddle_assembled() {translate([saddle_x,0,arm_top])saddle();}
module print_plate() {
    translate([10,19,0])body_print();
    translate([10,62,0])lid_print();
    translate([56,19,0])arm_print();
    translate([91,61,0])saddle();
}

module assembly() {
    color("#78529b")body();
    color("#b696cd")lid();
    color("#9571b3")arm();
    color("#51415f")saddle_assembled();
    if(show_electronics) {
        color("#278d90")pcb_proxy();
        color("#242027")sensor_proxy();
        // Illustrative potentiometer envelope, not an exact reverse-engineered part.
        color("#3374ce")translate([10,-4,front_z])cube([9,8,component_h]);
        color("#eee9e2")translate([-11,-6.5,front_z])cube([10.5,13,6]);
    }
}

if(part=="body")body_print();
else if(part=="lid")lid_print();
else if(part=="arm")arm_print();
else if(part=="saddle")saddle();
else if(part=="plate")print_plate();
else if(part=="assembly")assembly();
else if(part=="pcb_gauge")translate([0,0,-back_z])pcb_proxy();
else if(part=="check_body_lid")intersection(){body();lid();}
else if(part=="check_body_arm")intersection(){body();arm();}
else if(part=="check_lid_arm")intersection(){lid();arm();}
else if(part=="check_arm_saddle")intersection(){arm();saddle_assembled();}
else if(part=="check_body_pcb")intersection(){body();pcb_proxy();}
else if(part=="check_lid_pcb")intersection(){lid();pcb_proxy();}
else if(part=="check_body_sensor")intersection(){body();sensor_proxy();}
else if(part=="check_slider_travel") {
    for(position=[slot_start,saddle_x,slot_end])
        intersection() {
            translate([position,0,arm_top])saddle();
            union() {
                body();lid();arm();
                // Conservative M3 head + washer envelopes on the four cover bolts.
                for(x=cover_screw_x,y=cover_screw_y)
                    translate([x,y,arm_top])cylinder(d=7,h=4);
            }
        }
}
else assert(false,"Unknown part");
