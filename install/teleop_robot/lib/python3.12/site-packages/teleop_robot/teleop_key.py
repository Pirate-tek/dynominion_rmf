#!/usr/bin/env python3

import rclpy
from geometry_msgs.msg import TwistStamped
import sys, select, os
if os.name == 'nt':
  import msvcrt
else:
  import tty, termios


DYNO_AT_MAX_LIN_VEL = 1.0
DYNO_AT_MAX_ANG_VEL = 0.6

DYNO_SND_MAX_LIN_VEL = 1.0
DYNO_SND_MAX_ANG_VEL = 0.6

LIN_VEL_STEP_SIZE = 0.05
ANG_VEL_STEP_SIZE = 0.06

msg = """
Control Your dyno!
---------------------------
Moving around:
        w
   a    s    d
        x

w/x : increase/decrease linear velocity 
a/d : increase/decrease angular velocity 

space key, s : force stop

CTRL-C to quit
"""

e = """
Communications Failed
"""

settings = None
dyno_model = "AT100"

def getKey():
    global settings
    if os.name == 'nt':
      if sys.version_info[0] >= 3:
        return msvcrt.getch().decode()
      else:
        return msvcrt.getch()

    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''

    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def vels(target_linear_vel, target_angular_vel):
    return "currently:\tlinear vel %s\t angular vel %s " % (target_linear_vel,target_angular_vel)

def makeSimpleProfile(output, input, slop):
    if input > output:
        output = min( input, output + slop )
    elif input < output:
        output = max( input, output - slop )
    else:
        output = input

    return output

def makeBrakingProfile(output,input,steps,i):
      output = input - ((steps-i)*input/steps)
      return output

def constrain(input, low, high):
    if input < low:
      input = low
    elif input > high:
      input = high
    else:
      input = input

    return input

def checkLinearLimitVelocity(vel):
    global dyno_model
    if dyno_model == "AT100":
      vel = constrain(vel, -DYNO_AT_MAX_LIN_VEL, DYNO_AT_MAX_LIN_VEL)
    elif dyno_model == "SND100":
      vel = constrain(vel, -DYNO_SND_MAX_LIN_VEL, DYNO_SND_MAX_LIN_VEL)
    else:
      vel = constrain(vel, -DYNO_AT_MAX_LIN_VEL, DYNO_AT_MAX_LIN_VEL)

    return vel

def checkAngularLimitVelocity(vel):
    global dyno_model
    if dyno_model == "AT100":
      vel = constrain(vel, -DYNO_AT_MAX_ANG_VEL, DYNO_AT_MAX_ANG_VEL)
    elif dyno_model == "SND100":
      vel = constrain(vel, -DYNO_SND_MAX_ANG_VEL, DYNO_SND_MAX_ANG_VEL)
    else:
      vel = constrain(vel, -DYNO_AT_MAX_ANG_VEL, DYNO_AT_MAX_ANG_VEL)

    return vel

def main():
    global settings, dyno_model
    if os.name != 'nt':
        settings = termios.tcgetattr(sys.stdin)

    rclpy.init()
    node = rclpy.create_node('teleop_key')
    pub = node.create_publisher(TwistStamped, 'cmd_vel',10)

    node.declare_parameters(
            namespace='',
            parameters=[
                ('model', 'AT100'),
            ]
        )
    dyno_model = node.get_parameter('model').value

    status = 0
    target_linear_vel   = 0.0
    target_angular_vel  = 0.0
    control_linear_vel  = 0.0
    control_angular_vel = 0.0
    braking = False

    try:
        print(msg)
        while(1):
          key = getKey()
          if key == 'w' :
              braking = False
              target_linear_vel = checkLinearLimitVelocity(target_linear_vel + LIN_VEL_STEP_SIZE)
              # target_angular_vel=0.0
              # control_angular_vel = 0.0
              status = status + 1
              print(vels(target_linear_vel,target_angular_vel))
          elif key == 'x' :
              braking = False
              target_linear_vel = checkLinearLimitVelocity(target_linear_vel - LIN_VEL_STEP_SIZE)
              # target_angular_vel=0.0
              # control_angular_vel = 0.0
              status = status + 1
              print(vels(target_linear_vel,target_angular_vel))
          elif key == 'a' :
              braking = False
              target_angular_vel = checkAngularLimitVelocity(target_angular_vel + ANG_VEL_STEP_SIZE)
              # target_linear_vel =0.0
              # control_linear_vel  = 0.0
              status = status + 1
              print(vels(target_linear_vel,target_angular_vel))
          elif key == 'd' :
              braking = False
              target_angular_vel = checkAngularLimitVelocity(target_angular_vel - ANG_VEL_STEP_SIZE)
              # target_linear_vel =0.0
              # control_linear_vel  = 0.0
              status = status + 1
              print(vels(target_linear_vel,target_angular_vel))
          elif key == ' ' or key == 's' :
              braking = True
              steps = 5
              last_control_linear_vel = target_linear_vel
              last_control_angular_vel = target_angular_vel
              target_linear_vel   = 0.0
              control_linear_vel  = 0.0
              target_angular_vel  = 0.0
              control_angular_vel = 0.0
          else:
              if (key == '\x03'):
                  break

          if status == 20 :
              print(msg)
              status = 0

          twist = TwistStamped()
          twist.header.frame_id = 'base_footprint'
          if(braking == True):
          
            braking_steps = steps
            while(steps >= 0):
              control_braking_linear_vel = makeBrakingProfile(control_linear_vel, last_control_linear_vel,braking_steps,steps)
              twist.twist.linear.x = control_braking_linear_vel; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0

              control_braking_angular_vel = makeBrakingProfile(control_angular_vel, last_control_angular_vel,braking_steps,steps)
              twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = control_braking_angular_vel

              print(vels(control_braking_linear_vel, control_braking_angular_vel))

              twist.header.stamp = node.get_clock().now().to_msg()
              pub.publish(twist)
              steps = steps-1

            
            control_braking_linear_vel = 0.0
            control_braking_angular_vel = 0.0
            print(vels(control_braking_linear_vel, control_braking_angular_vel))
            twist.twist.linear.x = control_braking_linear_vel; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0
            twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = control_braking_angular_vel
            twist.header.stamp = node.get_clock().now().to_msg()
            pub.publish(twist)


          else:
            control_linear_vel = makeSimpleProfile(control_linear_vel, target_linear_vel, (LIN_VEL_STEP_SIZE/2.0))
            twist.twist.linear.x = control_linear_vel; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0

            control_angular_vel = makeSimpleProfile(control_angular_vel, target_angular_vel, (ANG_VEL_STEP_SIZE/2.0))
            twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = control_angular_vel
            twist.header.stamp = node.get_clock().now().to_msg()
            pub.publish(twist)

    except:
        print(e)

    finally:
        twist = TwistStamped()
        twist.header.frame_id = 'base_footprint'
        twist.twist.linear.x = 0.0; twist.twist.linear.y = 0.0; twist.twist.linear.z = 0.0
        twist.twist.angular.x = 0.0; twist.twist.angular.y = 0.0; twist.twist.angular.z = 0.0
        twist.header.stamp = node.get_clock().now().to_msg()
        pub.publish(twist)

    if os.name != 'nt':
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)

if __name__=="__main__":
    main()