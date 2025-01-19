Include: <../base-walls-min.g>

goal (floor){ shape:ssBox, Q:"t(-1.5 1.5 .0)", size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table} }

goal_visible (floor) 	{ shape:ssBox, Q:"t(-1.5 1.5 0.1)", size:[0.2 0.2 0.1 0.005], color:[1. .3 .3] }


wall1_v (world){ shape:ssBox, Q:"t(-1.0 1.5 0.3)", size:[0.1 0.9 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }
wall2_h (world){ shape:ssBox, Q:"t(-0.5 1. 0.3)", size:[0.9 .1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }
wall3_v (world){ shape:ssBox, Q:"t(-1 0.05 0.3)", size:[0.1 0.9 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }
wall4_h (world){ shape:ssBox, Q:"t(-0.5 0.5 0.3)", size:[0.9 .1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }


wall5_v (world){ shape:ssBox, Q:"t(-1.45 0.05 0.3)", size:[0.1 0.9 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }
wall6_h (world){ shape:ssBox, Q:"t(-1.7 0.5 0.3)", size:[0.5 .1 0.6 .02], color:[0.6953 0.515625 .453125],  contact: 1 }

wall8_h (world){ shape:ssBox, Q:"t(-1.75 1.0 0.3)", size:[0.5 .1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }

egoJoint(world){ Q:[0 0 0.1] } # works
obj(egoJoint) { type:ssBox size:[.4 .3 .2 .02] color:[0. 0. 1.],  logical={ gripper }, joint:transXY, limits: [-4 4 -4 4], contact: 1 }
