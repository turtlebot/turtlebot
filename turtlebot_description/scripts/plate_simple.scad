// valid for plates top and middle
translate([0, 0, -3])
  linear_extrude(height = 6)
    polygon([[130, 105], [130, -105], [20, -170], [-20, -170], [-157, -90], [-157, 90], [-20, 170], [20, 170]]);

/* original mesh to compare
translate([-100, 0, 0])
  rotate([90, 0, -90])
    scale(1000)
      import("../meshes/stacks/hexagons/plate_top.stl", convexity=3);
*/