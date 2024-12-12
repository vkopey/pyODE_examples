from numpy import fromstring
import ode
v=0
if v:
    import odeViz.ode_visualization as ode_viz
import sys, random

x=sys.stdin.read()
if x:
    x=x[1:-1]
    x=fromstring(x, dtype=float, sep=' ')
else:
    x=[0.10334363, 0.49620229, 0.30048395] #-8.28981301768

world = ode.World()
world.setGravity((0, -9.81, 0))
world.setLinearDamping(0.1)
world.setAngularDamping(0.1)

space = ode.Space()

ground = ode.GeomPlane(space, (0, 1, 0), 0)

def create_box(world, space, density, lx, ly, lz):
    body = ode.Body(world)
    M = ode.Mass()
    M.setBox(density, lx, ly, lz)
    body.setMass(M)
    body.shape = "box"
    body.boxsize = (lx, ly, lz)
    # Create a box geom for collision detection
    geom = ode.GeomBox(space, lengths=body.boxsize)
    geom.setBody(body)
    return body, geom

B,G=[],[]
for i in range(100):
    b,g=create_box(world, space, 700, 0.05,0.05,0.05)
    b.setPosition((random.uniform(-2,-1),random.uniform(0,0.5),random.uniform(-1,1)))
    b.setRotation([1,random.uniform(0,3),0, random.uniform(0,3),1,0, 0,0,1])
    B.append(b)
    G.append(g)

chassis = ode.Body(world)
M = ode.Mass()
M.setBox(10, 1, 0.2, 0.5)  # density, lx, ly, lz
chassis.setMass(M)
chassis.setPosition((0, 0.5, 0))

chassis_geom = ode.GeomBox(space, lengths=(1, 0.2, 0.5))
chassis_geom.setBody(chassis)

wheels = []
wheel_geoms = []
#wheel_positions = [(-0.5, 0, 0.3), (0.5, 0, 0.3), (-0.5, 0, -0.3), (0.5, 0, -0.3)]
wheel_positions = [(-x[1], 0, x[2]), (x[1], 0, x[2]), (-x[1], 0, -x[2]), (x[1], 0, -x[2])]
for pos in wheel_positions:
    wheel = ode.Body(world)
    M = ode.Mass()
    M.setSphere(1, x[0])  # density=1, radius=0.2
    #M.setCylinderTotal(0.1, 1, x[0], 0.05)
    wheel.setMass(M)
    wheel.setPosition((pos[0], pos[1] + 0.1, pos[2]))
    wheels.append(wheel)

    wheel_geom = ode.GeomSphere(space, radius=x[0]) #0.2
    #wheel_geom = ode.GeomCylinder(space, radius=x[0], length=0.05)
    wheel_geom.setBody(wheel)
    wheel_geoms.append(wheel_geom)

# Create joints to connect wheels to the chassis
joints = []
for i, wheel in enumerate(wheels):
    #joint = ode.HingeJoint(world)
    joint = ode.Hinge2Joint(world)
    joint.attach(chassis, wheel)
    joint.setAnchor(wheel.getPosition())
    #joint.setAxis((0, 0, 1))
    joint.setAxis1((0, 1, 0))
    joint.setAxis2((0, 0, 1))
    joint.setParam(ode.ParamSuspensionERP, 0.5)
    joint.setParam(ode.ParamSuspensionCFM, 0.8)
    joints.append(joint)

contact_group = ode.JointGroup()
# Collision callback function
def near_callback(args, geom1, geom2):
    # Check for collisions
    contacts = ode.collide(geom1, geom2)
    for contact in contacts:
        contact.setBounce(0.8)
        contact.setMu(5)
        joint = ode.ContactJoint(world, contact_group, contact)
        joint.attach(geom1.getBody(), geom2.getBody())

# Initialize the visualization
if v:
    viz = ode_viz.ODE_Visualization(world, [space], dt = 0.01)
    viz.GetActiveCamera().SetPosition(0,1,20)
    viz.update()

# Simulation loop
dt = 0.01
for i in range(1000):
    wheels[0].addTorque((0,0,2))
    wheels[2].addTorque((0,0,2))
    #joints[0].addTorque(-1)
    #joints[2].addTorque(-1)
    #wheels[1].setAngularVel((0,0,20))
    #wheels[3].setAngularVel((0,0,20))
    space.collide((world, contact_group), near_callback)
    world.step(dt)
    contact_group.empty()
    if v: viz.update()

#print np.array2string(x)
print chassis.getPosition()[0]
exit()
