import math 
def control(goal_x,goal_y,old_x,old_y,old_theta):

    error_d = math.sqrt((goal_x - old_x)**2 + (goal_y - old_y)**2)
    error_theta = math.atan2(goal_y-old_y, goal_x-old_x)-old_theta
    if error_theta > math.pi or error_theta < -math.pi:
        error_theta = (error_theta + math.pi) % (2*math.pi) - math.pi
    
    kp_d = 2
    kp_theta = 2
    velocity = kp_d * error_d
    angular_velocity = kp_theta * error_theta
    if abs(error_theta) > 0.5:
        velocity = 0
    return velocity, angular_velocity

def states(velocity,angular_velocity,dt,old_x,old_y,old_theta):
    new_theta = old_theta + angular_velocity*dt
    new_x = old_x + velocity * math.cos(new_theta)*dt
    new_y = old_y + velocity * math.sin (new_theta)*dt
    return (new_x,new_y,new_theta)

waypoints = []
while True:
 entry = input("enter the goal x and y(or done to stop): ")

 if entry == "done":
    break
 x,y = map(float, entry.split())
 waypoints.append((x,y))
 
dt = 0.1
old_x, old_y, old_theta = 0, 0, 0
trajectory = [ (old_x, old_y, old_theta) ]
step = 0
for waypoint in waypoints:     #waypoint is a tuple  
   a,b = waypoint
   while True:  
         velocity,angular_velocity = control(a,b,old_x,old_y,old_theta)
         step += 1
         vmax = 10
         wmax = 10
         velocity = max(min(velocity,vmax), -vmax)
         angular_velocity = max(min(angular_velocity,wmax), -wmax) 
         if math.sqrt((a - old_x)**2 + (b - old_y)**2) < 0.1:
              break 
     
         new_x, new_y, new_theta = states(velocity,angular_velocity,dt,old_x,old_y,old_theta)
         old_x, old_y, old_theta = new_x, new_y, new_theta
   trajectory.append((old_x, old_y, old_theta))
   print(trajectory)

import matplotlib.pyplot as plt

# Extract x and y from trajectory
xs = [s[0] for s in trajectory]
ys = [s[1] for s in trajectory]

# Extract waypoint coordinates
wx = [w[0] for w in waypoints]
wy = [w[1] for w in waypoints]

plt.figure(figsize=(8, 8))
plt.plot(xs, ys, 'b-', linewidth=2, label='Robot path')
plt.plot(0, 0, 'go', markersize=12, label='Start')
plt.scatter(wx, wy, c='red', s=100, zorder=5, label='Waypoints')
plt.plot(wx, wy, 'r--', alpha=0.3)  # line connecting waypoints

# Label each waypoint
for i, (x, y) in enumerate(waypoints):
    plt.annotate(f'W{i+1}', (x, y), 
                textcoords="offset points", 
                xytext=(10, 10), fontsize=9)

plt.grid(True, alpha=0.3)
plt.legend()
plt.axis('equal')
plt.title('Differential Drive Robot — Waypoint Following (P Controller)')
plt.xlabel('X (m)')
plt.ylabel('Y (m)')
plt.tight_layout()
plt.show()