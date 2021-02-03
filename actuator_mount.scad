hole_distance = 25;  // the distance of the mounting hole to the wall
mounting_plate_thickness = 5;
mounting_plate_height = 40;
mounting_plate_width = 60;

difference()
{
    union(){
            
        difference(){
            
            union()
            {    
                cube([35, 20, 20]);    
                translate([35,10,10])
                {
                    cylinder(20,10, 10, true, $fn=1000);
                }
            }
            translate([35, 10, 20])
            {
                cylinder(50, 5, 5, true, $fn=1000);
            }
        }

        translate([-5, -20, -20])
        {
            cube([mounting_plate_thickness, mounting_plate_height, mounting_plate_width]);  
        }



        translate([0, -14, 20])
        {
            rotate([0,90,0])
            {
                prism(20, 15, 38);
            }
        }
    }


    translate([0, 0, -10])
    {
            rotate([0,90,0])
            {
                cylinder(30 ,4, 4, true, $fn=1000);
            }
     }

   translate([0, 0, 30])
    {
            rotate([0,90,0])
            {
                cylinder(30 ,4, 4, true, $fn=1000);
            }
     }



}

module prism(l, w, h){
   polyhedron(
           points=[[0,0,0], [l,0,0], [l,w,0], [0,w,0], [0,w,h], [l,w,h]],
           faces=[[0,1,2,3],[5,4,3,2],[0,4,5,1],[0,3,4],[5,2,1]]
           );
   
}
