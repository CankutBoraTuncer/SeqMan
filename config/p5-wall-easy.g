Include: <base-walls-min.g>

egoJoint(world){ Q:[0.0 0.0 0.1] } # works
ego(egoJoint) {
    shape:ssCylinder, size:[.2 .2 .02], color:[0.1 0.3 0.5], logical:{gripper}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1, Q:[0.0 0.0 0.01]
}


goal (floor){ shape:ssBox, Q:"t(-1.2 1.3 .0)", size:[0.2 0.2 .1 .005], color:[1. .3 .3 0.9], contact:0, logical:{table} }
goal_visible (floor) 	{ shape:ssBox, Q:"t(-1.2 1.3 0.1)", size:[0.2 0.2 0.1 0.005], color:[1. .3 .3] }

wall2_h (world){ joint: rigid, shape:ssBox, Q:"t(-0.14 -1.2 0.3)", size:[2.7 .1 0.6 .02], color:[0.6953 0.515625 .453125],contact: 1 }
wall8_h (world){ joint: rigid,shape:ssBox, Q:"t(-1.45 1.0 0.3)", size:[1.0 .1 0.6 .02], color:[0.6953 0.515625 .453125], contact: 1 }

obj(floor) { type:ssBox size:[.3 .3 .2 .02] Q:"t(-0.9 -1.5  .15)" color:[0. 0. 1.],  logical={ object }, joint:rigid, contact: 1 }




