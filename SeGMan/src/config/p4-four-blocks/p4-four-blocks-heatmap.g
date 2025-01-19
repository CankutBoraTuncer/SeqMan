Include: <../base-walls-min-heatmap.g>

egoJoint(world){ Q:[-1.3 -1.3 0.1] }
ego(egoJoint) {
    shape:ssCylinder, size:[.2 .2 .02], color:[1 1 1], logical:{gripper}, limits: [-4 4 -4 4], sampleUniform:1,
    joint:transXY, contact: 1
}


goal (floor){ shape:ssBox, Q:"t(-1.5 1.5 .0)", size:[0.2 0.2 .1 .005], color:[1 1 1], contact:0, logical:{table} }

goal_visible (floor) 	{ shape:ssBox, Q:"t(-1.5 1.5 0.1)", size:[0.2 0.2 0.1 0.005], color:[1 1 1] }


obj1Joint(world){ Q:[0.0 0.0 0.1] } # works
obj1(obj1Joint) {
    shape:ssBox, Q:[-1.2 -0.3 .0], size:[1 0.2 .2 .02], logical={ object } nomass:1,  color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_r(floor) {
    shape:ssBox, Q:[-1.2 -0.3 .15], size:[1 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_hm_y(floor) {
    shape:ssBox, Q:[-1.2 -0.3 .15], size:[1.1 0.3 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj1_cam_g(world): { rel: [-1.2, -0.3, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },
obj1_cam_rel(obj1): { X: [-1.2, -0.3, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },

obj2Joint(world){ Q:[0.0 0.0 0.1] } # works
obj2(obj2Joint) {
    shape:ssBox, Q:[-0.75 -1.2 .0], size:[.2 1 .2 .02], logical={ object } nomass:1,  color:[0 1.0 0],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_r(floor) {
    shape:ssBox, Q:[-0.75 -1.2 .15], size:[.2 1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_hm_y(floor) {
    shape:ssBox, Q:[-0.75 -1.2 .15], size:[.3 1.1 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj2_cam_g(world): { rel: [-0.75, -1.2, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },
obj2_cam_rel(obj2): { X: [-0.75, -1.2, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },

obj4Joint(world){ Q:[0.0 0.0 0.1] } # works
obj4(obj4Joint) {
    shape:ssBox, Q:[-1.4 -0.75 .0], size:[1 0.2 .2 .02], logical={ object } nomass:1,  color:[1.0 0 0],
    joint:rigid, friction:.1  contact: 1
}
obj4_hm_r(floor) {
    shape:ssBox, Q:[-1.4 -0.75 .15], size:[1 0.2 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj4_hm_y(floor) {
    shape:ssBox, Q:[-1.4 -0.75 .15], size:[1.1 0.3 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj4_cam_g(world): { rel: [-1.4, -0.75, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },
obj4_cam_rel(obj4): { X: [-1.4, -0.75, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },


obj3Joint(world){ Q:[0.0 0.0 0.1] } # works
obj3(obj3Joint) {
    shape:ssBox, Q:[-0.4 -1.4 .0], size:[.2 1 .2 .02], logical={ object } nomass:1,  color:[0 1.0 1.0],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_r(floor) {
    shape:ssBox, Q:[-0.4 -1.4 .15], size:[.2 1 .6 .02], logical={ object } nomass:1,  color:[1 0 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_hm_y(floor) {
    shape:ssBox, Q:[-0.4 -1.4 .15], size:[.3 1.1 .4 .02], logical={ object } nomass:1,  color:[1 1 0 ],
    joint:rigid, friction:.1  contact: 1
}
obj3_cam_g(world): { rel: [-0.4, -1.4, 2,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },
obj3_cam_rel(obj3): { X: [-0.4, -1.4, 2.1,   0.00040292,  -0.99999992,   0.00000000,   0.00000000 ], shape: marker, size: [0.1], width: 300, height: 300, focalLength: 1, zRange: [0.5, 3]  },


objJoint(world){ Q:[0.0 0.0 0.1] } # works
obj(objJoint) {
    shape:ssBox, Q:[1.6 -0.5 .0], size:[.3 .3 0.2 .02], logical={ object }  nomass:1, color:[0 0 1.0],
    joint:rigid, friction:.1  contact: 1
}


