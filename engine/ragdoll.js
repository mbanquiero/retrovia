// helper global : Crea un ragdoll body

function RagDollBody(X,Y,k,an)
{
    // Ragdoll
    var shouldersDistance = 5*k;
    var upperArmLength = 4*k;
    var lowerArmLength = 4*k;
    var upperArmSize = 2*k;
    var lowerArmSize = 2*k;
    var neckLength = 0.5*k;
    var headRadius = 2.5*k;
    var upperBodyLength = 6*k;
    var pelvisLength = 4*k;
    var upperLegLength = 5*k;
    var upperLegSize = 2*k;
    var lowerLegSize = 2*k;
    var lowerLegLength = 5*k;

    var dist = 0.5*k;
    var r = 1*k;

    var i0 = _world.m_bodyCount;
    // Lower legs
    var lowerLeftLeg = _world.AddCircle(X - shouldersDistance / 2, Y - lowerLegLength / 2, r);
    var lowerRightLeg = _world.AddCircle(X+shouldersDistance/2,Y-lowerLegLength / 2,r);

    // Upper legs
    var upperLeftLeg = _world.AddCircle(X-shouldersDistance/2,lowerLeftLeg.position.y-lowerLegLength/2-upperLegLength/2,r);
    var upperRightLeg = _world.AddCircle(X+shouldersDistance/2,lowerRightLeg.position.y-lowerLegLength/2-upperLegLength/2,r);

    // Pelvis
    var pelvis = _world.AddCircle(X, upperLeftLeg.position.y-upperLegLength/2-pelvisLength/2,r);

    // Upper body
    var upperBody = _world.AddCircle(X,pelvis.position.y-pelvisLength/2-upperBodyLength/2,r);

    // Head
    var head = _world.AddCircle(X,upperBody.position.y-upperBodyLength/2-headRadius-neckLength,r);

    // Upper arms
    var upperLeftArm = _world.AddCircle(X-shouldersDistance/2-upperArmLength/2, upperBody.position.y-upperBodyLength/2,r);
    var upperRightArm = _world.AddCircle(X+shouldersDistance/2+upperArmLength/2, upperBody.position.y-upperBodyLength/2,r);

    // lower arms
    var lowerLeftArm = _world.AddCircle(upperLeftArm.position.x - lowerArmLength/2 - upperArmLength/2,upperLeftArm.position.y,r);
    var lowerRightArm = _world.AddCircle(upperRightArm.position.x + lowerArmLength/2 + upperArmLength/2,upperRightArm.position.y,r);

    // Neck joint
    var neckJoint = _world.AddJointConstraint(head, upperBody);

    // Knee joints
    var leftKneeJoint = _world.AddJointConstraint(lowerLeftLeg, upperLeftLeg);
    var rightKneeJoint = _world.AddJointConstraint(lowerRightLeg, upperRightLeg);

    // Hip joints
    var leftHipJoint = _world.AddJointConstraint(upperLeftLeg, pelvis);
    var rightHipJoint = _world.AddJointConstraint(upperRightLeg, pelvis);

    // Spine
    var spineJoint = _world.AddJointConstraint(pelvis, upperBody);

    // Shoulders
    var leftShoulder = _world.AddJointConstraint(upperBody, upperLeftArm);
    var rightShoulder= _world.AddJointConstraint(upperBody, upperRightArm);

    // Elbow joint
    var leftElbowJoint = _world.AddJointConstraint(lowerLeftArm, upperLeftArm);
    var rightElbowJoint = _world.AddJointConstraint(lowerRightArm, upperRightArm);

    // no quiero que dibuje cm ni orientacion en los circulos que representan las articulaciones
    for (var i = i0 ; i < _world.m_bodyCount; ++i)
    {
        _world.bodies[i].shape.drawCM = false;
        _world.bodies[i].shape.drawOrient = false;
        _world.bodies[i].color = '#ff33ff';

    }


    // constrains virtuales
    var c = _world.AddJointConstraint(lowerLeftLeg, head);
    c.visible = 0;
    c = _world.AddJointConstraint(lowerRightLeg, head);
    c.visible = 0;
    c = _world.AddJointConstraint(upperLeftArm, head);
    c.visible = 0;
    c = _world.AddJointConstraint(upperRightArm, head);
    c.visible = 0;
	
	// tiene orientacion
	if(an!=0)
	{
		var cg = new Vector2(X,Y);
		for (var i = i0; i < _world.m_bodyCount; ++i) 
		{
			var B = _world.bodies[i];
			var p = vec2_substract(B.position , cg);
			p.rotate(an);
			B.position = vec2_add(cg , p );
		}
	}
}
