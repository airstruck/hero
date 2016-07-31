local lp = love.physics

local function removeDestroyed (t)
    if not t then return {} end
    for i = #t, 1, -1 do
        if t[i]:isDestroyed() then
            table.remove(t, i)
        end
    end
    return t
end

-- Shape

local function ChainShape (t)
    local shape = lp.newChainShape(false, t.points)
    if #t.nextVertex > 1 then
        shape:setNextVertex(t.nextVertex[1], t.nextVertex[2])
    end
    if #t.previousVertex > 1 then
        shape:setPreviousVertex(t.previousVertex[1], t.previousVertex[2])
    end
    return shape
end

local function CircleShape (t)
    return lp.newCircleShape(t.point[1], t.point[2], t.radius)
end

local function EdgeShape (t)
    return lp.newEdgeShape(t.points[1], t.points[2], t.points[3], t.points[4])
end

local function PolygonShape (t)
    return lp.newPolygonShape(t.points)
end

local shapeByType = {
    chain = ChainShape, 
    circle = CircleShape,
    edge = EdgeShape,
    polygon = PolygonShape,
}

local function Shape (t, body)
    local shape = shapeByType[t.type](t, body)
    
    t.shape = shape
    
    return shape
end

local function ChainShapeState (shape)
    return {
        points = { shape:getPoints() },
        nextVertex = { shape:getNextVertex() },
        previousVertex = { shape:getPreviousVertex() },
    }
end

local function CircleShapeState (shape)
    return {
        point = { shape:getPoint() },
        radius = shape:getRadius(),
    }
end

local function EdgeShapeState (shape)
    return {
        points = { shape:getPoints() },
    }
end

local function PolygonShapeState (shape)
    return {
        points = { shape:getPoints() },
    }
end

local shapeStateByType = {
    chain = ChainShapeState, 
    circle = CircleShapeState,
    edge = EdgeShapeState,
    polygon = PolygonShapeState,
}

local function ShapeState (shape)
    local shapeType = shape:getType()
    local t = shapeStateByType[shapeType](shape)
    
    t.type = shapeType
    
    return t
end

-- Fixture

local function Fixture (t, body)
    local shape = Shape(t.shapeState)
    local fixture = lp.newFixture(body, shape)
    
    fixture:setCategory(t.category)
    fixture:setDensity(t.density)
    fixture:setFilterData(t.filterData[1], t.filterData[2], t.filterData[3])
    fixture:setFriction(t.friction)
    fixture:setGroupIndex(t.groupIndex)
    fixture:setMask(unpack(t.mask))
    fixture:setRestitution(t.restitution)
    fixture:setSensor(t.sensor)
    fixture:setUserData(t.userData)
    
    body:resetMassData()
    
    t.fixture = fixture
    
    return fixture
end

local function FixtureState (fixture)
    return {
        id = tostring(fixture),
        category = fixture:getCategory(),
        density = fixture:getDensity(),
        filterData = { fixture:getFilterData() }, -- categories, mask, group
        friction = fixture:getFriction(),
        groupIndex = fixture:getGroupIndex(),
        mask = { fixture:getMask() },
        restitution = fixture:getRestitution(),
        sensor = fixture:isSensor(),
        userData = fixture:getUserData(),
        
        shapeState = ShapeState(fixture:getShape()),
    }
end

-- Body

local function Body (t, world)
    local body = lp.newBody(world, t.x, t.y, t.type)

    body:setActive(t.active)
    body:setAngle(t.angle)
    body:setAngularDamping(t.angularDamping)
    body:setAngularVelocity(t.angularVelocity)
    body:setAwake(t.awake)
    body:setBullet(t.bullet)
    body:setFixedRotation(t.fixedRotation)
    body:setGravityScale(t.gravityScale)
    body:setInertia(t.inertia)
    body:setLinearDamping(t.linearDamping)
    body:setLinearVelocity(t.linearVelocity[1], t.linearVelocity[2])
    body:setMass(t.mass)
    -- body:setMassData(t.massData[1], t.massData[2], t.massData[3], t.massData[4])
    body:setSleepingAllowed(t.sleepingAllowed)
    --body:setType(t.type)
    body:setUserData(t.userData)
    --body:setX(t.x)
    --body:setY(t.y)
    
    for i, fixtureState in ipairs(t.fixtureStates) do
        Fixture(fixtureState, body)
    end
    
    t.body = body
    
    return body
end

local function BodyState (body)
    local fixtureStates = {}
    
    local fixtures = removeDestroyed(body:getFixtureList())
    
    for i, fixture in ipairs(fixtures) do
        fixtureStates[i] = FixtureState(fixture)
    end
    
    return {
        id = tostring(body),
        -- members
        active = body:isActive(),
        angle = body:getAngle(),
        angularDamping = body:getAngularDamping(),
        angularVelocity = body:getAngularVelocity(),
        awake = body:isAwake(),
        bullet = body:isBullet(),
        fixedRotation = body:isFixedRotation(),
        gravityScale = body:getGravityScale(),
        inertia = body:getInertia(),
        linearDamping = body:getLinearDamping(),
        linearVelocity = { body:getLinearVelocity() },
        mass = body:getMass(),
        massData = { body:getMassData() }, -- x, y, mass, inertia
        -- position = { body:getPosition() },
        sleepingAllowed = body:isSleepingAllowed(),
        type = body:getType(),
        userData = body:getUserData(),
        x = body:getX(),
        y = body:getY(),
        -- children
        fixtureStates = fixtureStates,
    }
end

-- Joint

local function DistanceJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, collideConnected
    local joint = lp.newDistanceJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.collideConnected)
        
    joint:setDampingRatio(t.dampingRatio)
    joint:setFrequency(t.frequency)
    joint:setLength(t.length)
    
    return joint
end

local function FrictionJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, collideConnected
    local joint = lp.newFrictionJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.collideConnected)
        
    joint:setMaxForce(t.maxForce)
    joint:setMaxTorque(t.maxTorque)
    
    return joint
end

local function GearJoint (t, bodyMap, jointMap)
    -- nyi
end

local function MotorJoint (t, bodyMap, jointMap)
    -- body1, body2, correctionFactor, collideConnected
    local joint = lp.newMotorJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.correctionFactor,
        t.collideConnected)
        
    joint:setAngularOffset(t.angularOffset)
    joint:setLinearOffset(t.linearOffset[1], t.linearOffset[2])
    joint:setMaxForce(t.maxForce)
    joint:setMaxTorque(t.maxTorque)
    
    return joint
end

local function MouseJoint (t, bodyMap, jointMap)
    -- body, x, y
    local joint = lp.newMouseJoint(
        bodyMap[t.bodies[1]].body,
        t.target[1], t.target[2])
        
    joint:setDampingRatio(t.dampingRatio)
    joint:setFrequency(t.frequency)
    joint:setMaxForce(t.maxForce)
    
    return joint
end

local function PrismaticJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, ax, ay, collideConnected
    local joint = lp.newPrismaticJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.axis[1], t.axis[2],
        t.collideConnected)

    joint:setLowerLimit(t.lowerLimit)
    joint:setMaxMotorForce(t.maxMotorForce)
    joint:setMotorEnabled(t.motorEnabled)
    joint:setMotorSpeed(t.motorSpeed)
    joint:setUpperLimit(t.upperLimit)
    joint:setLimitsEnabled(t.limitsEnabled)
    
    return joint
end

local function PulleyJoint (t, bodyMap, jointMap)
    -- body1, body2, gx1, gy1, gx2, gy2, x1, y1, x2, y2, ratio, collideConnected
    local joint = lp.newPulleyJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.groundAnchors[1], t.groundAnchors[2],
        t.groundAnchors[3], t.groundAnchors[4],
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.ratio,
        t.collideConnected)
    
    return joint
end

local function RevoluteJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, collideConnected
    local joint = lp.newRevoluteJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.collideConnected)
    
    joint:setLowerLimit(t.lowerLimit)
    joint:setMaxMotorTorque(t.maxMotorTorque)
    joint:setMotorEnabled(t.motorEnabled)
    joint:setMotorSpeed(t.motorSpeed)
    joint:setUpperLimit(t.upperLimit)
    joint:setLimitsEnabled(t.limitsEnabled)
    
    return joint
end

local function RopeJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, maxLength, collideConnected
    local joint = lp.newRopeJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.maxLength,
        t.collideConnected)
    
    return joint
end

local function WeldJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, collideConnected
    local joint = lp.newWeldJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.collideConnected)
    
    joint:setDampingRatio(t.dampingRatio)
    joint:setFrequency(t.frequency)
    
    return joint
end

local function WheelJoint (t, bodyMap, jointMap)
    -- body1, body2, x1, y1, x2, y2, ax, ay, collideConnected
    local joint = lp.newWheelJoint(
        bodyMap[t.bodies[1]].body, bodyMap[t.bodies[2]].body,
        t.anchors[1], t.anchors[2], t.anchors[3], t.anchors[4],
        t.axis[1], t.axis[2],
        t.collideConnected)

    joint:setLowerLimit(t.lowerLimit)
    joint:setMaxMotorTorque(t.maxMotorTorque)
    joint:setMotorEnabled(t.motorEnabled)
    joint:setMotorSpeed(t.motorSpeed)
    joint:setUpperLimit(t.upperLimit)
    joint:setSpringDampingRatio(t.springDampingRatio)
    joint:setSpringFrequency(t.springFrequency)
    joint:setLimitsEnabled(t.limitsEnabled)
    
    return joint
end

local jointByType = {
    distance = DistanceJoint, 
    friction = FrictionJoint,
    gear = GearJoint,
    motor = MotorJoint,
    mouse = MouseJoint,
    prismatic = PrismaticJoint,
    pulley = PulleyJoint,
    revolute = RevoluteJoint,
    rope = RopeJoint,
    weld = WeldJoint,
    wheel = WheelJoint,
}

local function Joint (t, bodyMap, jointMap)
    local joint = jointByType[t.type](t, bodyMap, jointMap)
    
    joint:setUserData(t.userData)
    
    t.joint = joint
    
    return joint
end

-- create joint states by joint type

local function DistanceJointState (joint, bodyMap, jointMap)
    return {
        dampingRatio = joint:getDampingRatio(),
        frequency = joint:getFrequency(),
        length = joint:getLength(),
    }
end

local function FrictionJointState (joint, bodyMap, jointMap)
    return {
        maxForce = joint:getMaxForce(),
        maxTorque = joint:getMaxTorque(),
    }
end

local function GearJointState (joint, bodyMap, jointMap)
    local jointA, jointB = joint:getJoints()
    return {
        joints = { jointMap[jointA], jointMap[jointB] },
        ratio = joint:getRatio(),
    }
end

local function MotorJointState (joint, bodyMap, jointMap)
    return {
        angularOffset = joint:getAngularOffset(),
        linearOffset = { joint:getLinearOffset() },
        maxForce = joint:getMaxForce(),
        maxTorque = joint:getMaxTorque(),
        correctionFactor = joint:getCorrectionFactor(),
    }
end	

local function MouseJointState (joint, bodyMap, jointMap)
    return {
        dampingRatio = joint:getDampingRatio(),
        frequency = joint:getFrequency(),
        maxForce = joint:getMaxForce(),
        target = { joint:getTarget() },
    }
end	

local function PrismaticJointState (joint, bodyMap, jointMap)
    return {
        axis = { joint:getAxis() },
        lowerLimit = joint:getLowerLimit(),
        maxMotorForce = joint:getMaxMotorForce(),
        motorSpeed = joint:getMotorSpeed(),
        upperLimit = joint:getUpperLimit(),
        limitsEnabled = joint:hasLimitsEnabled(),
        motorEnabled = joint:isMotorEnabled(),
    }
end

local function PulleyJointState (joint, bodyMap, jointMap)
    return {
        groundAnchors = { joint:getGroundAnchors() },
        ratio = joint:getRatio(),
    }
end	

local function RevoluteJointState (joint, bodyMap, jointMap)
    return {
        lowerLimit = joint:getLowerLimit(),
        maxMotorTorque = joint:getMaxMotorTorque(),
        motorSpeed = joint:getMotorSpeed(),
        upperLimit = joint:getUpperLimit(),
        limitsEnabled = joint:hasLimitsEnabled(),
        motorEnabled = joint:isMotorEnabled(),
    }
end

local function RopeJointState (joint, bodyMap, jointMap)
    return {
        maxLength = joint:getMaxLength(),
    }
end

local function WeldJointState (joint, bodyMap, jointMap)
    return {
        dampingRatio = joint:getDampingRatio(),
        frequency = joint:getFrequency(),
    }
end	

local function WheelJointState (joint, bodyMap, jointMap)
    return {
        axis = { joint:getAxis() },
        maxMotorTorque = joint:getMaxMotorTorque(),
        motorSpeed = joint:getMotorSpeed(),
        springDampingRatio = joint:getSpringDampingRatio(),
        springFrequency = joint:getSpringFrequency(),
        motorEnabled = joint:isMotorEnabled(),
    }
end

local jointStateByType = {
    distance = DistanceJointState, 
    friction = FrictionJointState,
    gear = GearJointState,
    motor = MotorJointState,
    mouse = MouseJointState,
    prismatic = PrismaticJointState,
    pulley = PulleyJointState,
    revolute = RevoluteJointState,
    rope = RopeJointState,
    weld = WeldJointState,
    wheel = WheelJointState,
}

local function JointState (joint, bodyMap, jointMap)
    local t = jointStateByType[joint:getType()](joint, bodyMap, jointMap)
    
    if bodyMap then
        local bodyA, bodyB = joint:getBodies()
        t.bodies = { bodyMap[bodyA], bodyMap[bodyB] }
    end
    
    t.anchors = { joint:getAnchors() }
    t.collideConnected = joint:getCollideConnected()
    t.type = joint:getType()
    t.userData = joint:getUserData()
    t.id = tostring(joint)
    
    return t
end

-- World

local function World (t, world)
    local bodyMap, jointMap = {}, {}
    local lookup = {}
    
    world = world or lp.newWorld()
    world:setGravity(t.gravity[1], t.gravity[2])
    world:setSleepingAllowed(t.sleepingAllowed)
    
    -- index all bodies and add them to the world
    for i, bodyState in ipairs(t.bodyStates) do
        bodyMap[i] = bodyState
        lookup[bodyState.id] = Body(bodyState, world)
        for i, fixtureState in ipairs(bodyState.fixtureStates)do
            lookup[fixtureState.id] = fixtureState.fixture
        end
    end
    
    -- first pass over joints; index them all
    for i, jointState in ipairs(t.jointStates) do
        jointMap[i] = jointState
    end
    
    -- second pass over joints; add them to the world
    for i, jointState in ipairs(t.jointStates) do
        lookup[jointState.id] = Joint(jointState, bodyMap, jointMap)
    end
    
    return world, lookup
end

local function sortGears (a, b)
    return (a:getType() ~= 'gear' and b:getType() == 'gear')
        or tostring(a) < tostring(b)
end

local function WorldState (world)
    local bodies = removeDestroyed(world:getBodyList())
    local joints = removeDestroyed(world:getJointList()) 
    local bodyStates, bodyMap, jointStates, jointMap = {}, {}, {}, {}
    
    for i, body in ipairs(bodies) do
        bodyMap[body] = i
        bodyStates[i] = BodyState(body)
    end
    
    table.sort(joints, sortGears)
    
    for i, joint in ipairs(joints) do
        jointMap[joint] = i
    end
    
    for i, joint in ipairs(joints) do
        jointStates[i] = JointState(joint, bodyMap, jointMap)
    end
    
    return {
        -- members
        gravity = { world:getGravity() },
        sleepingAllowed = world:isSleepingAllowed(),
        -- children
        bodyStates = bodyStates,
        jointStates = jointStates,
    }
end

return {
    Body = Body,
    BodyState = BodyState,
    Fixture = Fixture,
    FixtureState = FixtureState,
    Joint = Joint,
    JointState = JointState,
    World = World,
    WorldState = WorldState,
    load = World,
    save = WorldState,
}
