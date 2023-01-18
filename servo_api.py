import time
import struct

import can


class Servo(object):
    
    def __init__(self):
        
        # -- parameters --
        self.channel = 'COM7@3000000'  # change to relevant com port. lab comp is COM7.
        self.bitrate = 1000000
        
        # -- generals --
        self.null_byte = 0x00
        self.identifier = 0x140  # dont change
        
        self.read_pid_cmd_byte = 0x30
        self.read_single_circle_cmd_byte = 0x94
        self.read_multi_turn_cmd_byte = 0x92
        self.motor_off_cmd_byte = 0x80
        self.motor_stop_cmd_byte = 0x81
        self.motor_resume_cmd_byte = 0x88
        self.position_control_cmd_byte = 0xA4  # this requires commanding goal angle at a commanded speed
        
        self.angle_sensitivity = 0.01  # deg per LSB
        self.max_angular_speed_sensitivity = 1  # deg/s per LSB. this is specific to the max speed in the position commands.
        
        # ----
        self.bus = None
    
    
    def connect(self):
        '''
        call this first, externally.
        
        NOTE: this is specific to the RMD x8 servo.
        we use the USB-CAN Plus adapter from VScom. run their USB tool to configure the port, update firmware, etc.
        '''

        self.bus = can.interface.Bus(bustype='slcan',  # uses serial CAN, but there a usb2can interfact defined in https://python-can.readthedocs.io/en/stable/configuration.html
                                    channel=self.channel,
                                    rtscts=True,  # required per usbcan manual
                                    bitrate=self.bitrate)
        
        print(f'connection: {self.bus.state}.')
        
        return None
    
    def disconnect(self):
        '''
        call this last, externally.
        '''

        self.bus.shutdown()
        
        print(f'servo disconnected properly.')
        
        return None
    
    # -- commands --
    def read_pid_params(self, servo_id, timeout=10):

        # send the read_pid command
        self._send_msg_to_servo(servo_id, self.read_pid_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            cmd_byte = msg.data[0]

            # position loop (angle of servo)
            position_Kp = msg.data[2]
            position_Ki = msg.data[3]

            speed_Kp = msg.data[4]
            speed_Ki = msg.data[5]

            torque_Kp = msg.data[6]
            torque_Ki = msg.data[7]
            return (position_Kp, position_Ki, speed_Kp, speed_Ki, torque_Kp, torque_Ki)
        else:
            print('[!] timeout on PID read cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return (None, None, None, None, None, None)
        
    def read_single_circle_angle(self, servo_id, timeout=10):
        # technically the "read single-circle angle"

        # send the read_angle command byte
        self._send_msg_to_servo(servo_id, self.read_single_circle_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            cmd_byte = msg.data[0]

            #msg.data[1]  # null
            #msg.data[2]
            #msg.data[3]
            #msg.data[4]
            #msg.data[5]

            single_angle_low = msg.data[6]
            single_angle_high = msg.data[7]
            return (single_angle_low, single_angle_high)
        else:
            print('[!] timeout on read_single_circle_angle cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return (None, None)
    
    def read_multi_turn_angle(self, servo_id, timeout=10):

        # send the read_angle command byte
        self._send_msg_to_servo(servo_id, self.read_multi_turn_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            cmd_byte = msg.data[0]

            angle_low_byte1 = msg.data[1]
            
            angle_byte2 = msg.data[2]
            angle_byte3 = msg.data[3]
            angle_byte4 = msg.data[4]
            angle_byte5 = msg.data[5]
            angle_byte6 = msg.data[6]
            angle_byte7 = msg.data[7]
            
            return (angle_low_byte1, angle_byte2, angle_byte3, angle_byte4, angle_byte5, angle_byte6, angle_byte7)
        else:
            print('[!] timeout on read_multi_turn_angle cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return (None, None, None, None, None, None, None)
        
    def stop_motor(self, servo_id, timeout=10):

        # send the stop command byte
        self._send_msg_to_servo(servo_id, self.motor_stop_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            print('motor stopped.')
            
            return None
        else:
            print('[!] timeout on stop_motor cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return None
        
    def resume_motor(self, servo_id, timeout=10):

        # send the resume command byte
        self._send_msg_to_servo(servo_id, self.motor_resume_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            print('motor resumed.')
            
            return None
        else:
            print('[!] timeout on resume_motor cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return None
        
    def turn_motor_off(self, servo_id, timeout=10):

        # send the off command byte
        self._send_msg_to_servo(servo_id, self.motor_off_cmd_byte)
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            print('motor turned off, commands/status cleared.')
            
            return None
        else:
            print('[!] timeout on turn_motor_off cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return None
        
    def go_to_angle(self, servo_id, desired_angle, max_speed, timeout=10):
        '''
        input: servo_id to command (int), desired_angle (deg), max_speed (deg/s)
        output: None
        
        max_speed must be an integer, so we round to nearest int.
        '''

        # send the position control command
        self._send_position_control_to_servo(servo_id, round(desired_angle,2), round(max_speed))
        
        # read the message
        msg = self.bus.recv(timeout=timeout)

        if msg:
            cmd_byte = msg.data[0]

            motor_temp = msg.data[1]
            
            torque_current_low_byte = msg.data[2]
            torque_current_high_byte = msg.data[3]
            
            speed_low_byte = msg.data[4]
            speed_high_byte = msg.data[5]
            
            angle_low_byte = msg.data[6]
            angle_high_byte = msg.data[7]
            
            return (motor_temp, torque_current_low_byte, torque_current_high_byte, speed_low_byte, speed_high_byte, angle_low_byte, angle_high_byte)
        else:
            print('[!] timeout on go_to_angle cmnd [!]')
            # TODO: raise exception here? then shutdown bus?
            return (None, None, None, None, None, None, None)
        
        
    # -- internal methods --
    def _send_msg_to_servo(self, servo_id, cmd_byte):

        # no messages are extended frame (all standard)
        # 8 byte DLC.
        # identifier: 0x140+Id(1-32)

        msg = can.Message(arbitration_id=self.identifier+servo_id,
                          is_extended_id=False,
                          data=[cmd_byte,
                                self.null_byte,
                                self.null_byte,
                                self.null_byte,
                                self.null_byte,
                                self.null_byte,
                                self.null_byte,
                                self.null_byte])
        self.bus.send(msg)

        return None
    
    def _send_position_control_to_servo(self, servo_id, desired_angle, max_speed):
        
        # < means little-endian, I means 32-bit unsigned
        angle_byte_array = struct.pack("<I", int(desired_angle/self.angle_sensitivity))
        #  H means 16-bit.
        angular_speed_byte_array = struct.pack("<H", int(max_speed/self.max_angular_speed_sensitivity))

        msg = can.Message(arbitration_id=self.identifier+servo_id,
                          is_extended_id=False,
                          data=[self.position_control_cmd_byte,
                                self.null_byte,
                                angular_speed_byte_array[0],
                                angular_speed_byte_array[1],
                                angle_byte_array[0],
                                angle_byte_array[1],
                                angle_byte_array[2],
                                angle_byte_array[3]])
        self.bus.send(msg)

        return None
        
        
       
