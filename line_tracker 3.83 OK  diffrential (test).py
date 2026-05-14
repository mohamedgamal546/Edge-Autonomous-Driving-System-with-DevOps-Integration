import cv2
import numpy as np
import time
import logging
import serial
import serial.tools.list_ports
import threading
import collections  # Add this line

# Configure logging
logging.basicConfig(level=logging.INFO, 
                    format='%(asctime)s - %(levelname)s - %(message)s')
logger = logging.getLogger('LineFollower')


class SequenceHandler:
    """Class to handle the obstacle avoidance sequence using precise step movements"""
    
    def __init__(self, robot):
        self.robot = robot
        self.sequence_running = False
        self.current_step = 0
        self.last_command_time = 0
        self.command_delay = 7.0  # Increased delay to ensure commands complete
        self.command_sent = False  # Flag to track if current step has been sent
        
        # Define the obstacle avoidance sequence steps
        # Format: (left_steps, right_steps, left_speed, right_speed, description)
        # - Positive steps move forward for that wheel
        # - Negative steps move backward for that wheel
        # - Different step values for left/right wheels create turns
        self.sequence_steps = [
            (600, 600, 100, 100, "Turn Right"),    # Right turn (left wheel backward, right wheel forward)
            (600, -600, 100, 100, "Forward"),        # Move forward (both wheels forward)
            (-600, -600, 100, 100, "Turn Left"),     # Left turn (left wheel forward, right wheel backward)
            (1800, -1800, 100, 100, "Forward"),      # Longer forward movement
            (-600, -600, 100, 100, "Turn Left"),     # Move backward (both wheels backward)
            (600, -600, 100, 100, "Forward"),         # Final forward movement
            (600, 600, 100, 100, "Turn Right")    # Move backward (both wheels backward)
        ]
    
    def start_sequence(self, speed=150):
        """Start the obstacle avoidance sequence"""
        if not self.sequence_running:
            logger.info(f"Starting obstacle avoidance sequence with speed {speed}")
            
            # Check if robot is connected first
            if not self.robot.connected:
                logger.error("Cannot start sequence: Robot not connected")
                return False
                
            self.sequence_running = True
            self.current_step = 0
            self.last_command_time = 0  # Set to 0 to send first command immediately
            self.command_sent = False   # Reset command sent flag
            
            # Update speeds if specified
            if speed > 0:
                for i in range(len(self.sequence_steps)):
                    left_steps, right_steps, _, _, desc = self.sequence_steps[i]
                    self.sequence_steps[i] = (left_steps, right_steps, speed, speed, desc)
            
            # Force immediate update to send first command
            self.update()
            
            return True
        return False
    
    def update(self):
        """Update the sequence progress - should be called regularly in the main loop"""
        if not self.sequence_running:
            return False
            
        current_time = time.time()
        
        # Check if enough time has passed to move to the next step
        if self.command_sent and (current_time - self.last_command_time > self.command_delay):
            # Time to move to the next step
            self.current_step += 1
            self.command_sent = False  # Reset flag for the new step
            
            # Check if we've completed the sequence
            if self.current_step >= len(self.sequence_steps):
                logger.info("Obstacle avoidance sequence completed")
                self.sequence_running = False
                
                # Send a stop command to ensure robot stops after sequence
                self.robot.send_command('S')
                
                return False
        
        # Send the current step command if it hasn't been sent yet
        if not self.command_sent:
            result = self._send_next_step()
            
            if result:
                # Mark this step as sent and update the timestamp
                self.command_sent = True
                self.last_command_time = current_time
            else:
                logger.error(f"Failed to send command for step {self.current_step+1}. Stopping sequence.")
                self.sequence_running = False
                return False
            
        return True
    
    def _send_next_step(self):
        """Send the next step command to the robot with improved error handling"""
        if not (0 <= self.current_step < len(self.sequence_steps)):
            logger.error(f"Step index out of range: {self.current_step}")
            return False
            
        # Extract step parameters
        left_steps, right_steps, left_speed, right_speed, desc = self.sequence_steps[self.current_step]
        
        # Create the position control command
        command = f'P{left_steps},{right_steps},{left_speed},{right_speed}'
        
        # Log the command being sent
        logger.info(f"Sequence step {self.current_step+1}/{len(self.sequence_steps)}: {desc} - {command}")
        
        # Clear last_command in robot to force sending even if similar to previous command
        old_last_command = None
        if hasattr(self.robot, 'last_command'):
            old_last_command = self.robot.last_command
            self.robot.last_command = None
        
        # Send the command
        result = self.robot.send_command(command)
        
        # Restore last_command if we had to clear it
        if old_last_command is not None:
            self.robot.last_command = command
        
        # Check if command was sent successfully
        if not result:
            logger.error(f"Failed to send command: {command}")
            return False
            
        # Send an E command to ensure motors are enabled
        if self.current_step == 0:
            time.sleep(0.1)  # Short delay
            self.robot.send_command('E')  # Enable motors
            
        return True
    
    def is_running(self):
        """Check if the sequence is currently running"""
        return self.sequence_running
    
    def get_current_step_info(self):
        """Get information about the current step safely"""
        if not self.sequence_running:
            return (0, 0, 0, 0, "Not Running")
            
        if 0 <= self.current_step < len(self.sequence_steps):
            return self.sequence_steps[self.current_step]
        
        # Return default if step is out of range
        return (0, 0, 0, 0, "Unknown Step")
    
    def stop(self):
        """Stop the current sequence"""
        if self.sequence_running:
            self.sequence_running = False
            
            # Send stop command
            self.robot.send_command('S')
            
            logger.info("Obstacle avoidance sequence stopped")
            return True
        return False

class BluetoothRobot:
    """Class to handle Bluetooth communication with the robot - updated for differential steering"""
    
    def __init__(self, port=None, baud_rate=9600, timeout=1):
        self.port = port
        self.baud_rate = baud_rate
        self.timeout = timeout
        self.serial = None
        self.connected = False
        self.lock = threading.Lock()
        self.last_command_time = 0
        self.command_cooldown = 0.1  # Reduced to 100ms since differential steering is smoother
        self.last_command = None     # Track the last command sent to avoid redundancy
        
    def list_available_ports(self):
        """List all available serial ports"""
        ports = []
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'hardware_id': port.hwid
            })
        return ports
        
    def connect(self, port=None):
        """Connect to the specified port or the stored port"""
        if port:
            self.port = port
            
        if not self.port:
            logger.error("No port specified for Bluetooth connection")
            return False
            
        try:
            logger.info(f"Attempting to connect to {self.port} at {self.baud_rate} baud")
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baud_rate,
                timeout=self.timeout
            )
            self.connected = True
            logger.info(f"Successfully connected to {self.port}")
            
            # Start the read thread
            self.running = True
            self.read_thread = threading.Thread(target=self._read_thread)
            self.read_thread.daemon = True
            self.read_thread.start()
            
            return True
        except Exception as e:
            logger.error(f"Failed to connect to {self.port}: {e}")
            self.connected = False
            return False
            
    def _read_thread(self):
        """Background thread to read responses from the robot"""
        while self.running and self.connected:
            try:
                if self.serial and self.serial.in_waiting > 0:
                    response = self.serial.readline().decode('utf-8').strip()
                    if response:
                        logger.info(f"Received from robot: {response}")
            except Exception as e:
                logger.error(f"Error reading from serial port: {e}")
                time.sleep(0.1)  # Prevent tight loop on error
    
    def send_command(self, command, force=False):
        """
        Send a command to the robot with improved reliability
        
        Args:
            command: The command to send
            force: If True, send even if identical to last command
        """
        if not self.connected or not self.serial:
            logger.warning(f"Cannot send command '{command}' - robot not connected")
            return False
        
        # Skip if identical to last command (unless force=True)
        if not force and command == self.last_command:
            logger.debug(f"Skipping duplicate command: {command}")
            return True
            
        try:
            # Check cooldown
            current_time = time.time()
            if force or current_time - self.last_command_time > self.command_cooldown:
                with self.lock:
                    # Add newline if not present
                    if not command.endswith('\n'):
                        command += '\n'
                    
                    # Send the command
                    self.serial.write(command.encode())
                    self.serial.flush()  # Add flush to ensure command is sent immediately
                    self.last_command_time = current_time
                    self.last_command = command.strip()
                    logger.info(f"Sent command: {command.strip()}")
                return True
            else:
                logger.debug(f"Command '{command}' throttled (cooldown)")
                return False
        except Exception as e:
            logger.error(f"Error sending command '{command}': {e}")
            return False
    
    def disconnect(self):
        """Disconnect from the robot"""
        self.running = False
        
        if self.serial:
            try:
                # Send stop command before disconnecting
                self.send_command('M0,0')  # Updated to differential steering stop
                time.sleep(0.1)  # Give time for command to be sent
                
                self.serial.close()
                logger.info("Disconnected from robot")
            except Exception as e:
                logger.error(f"Error disconnecting: {e}")
                
        self.connected = False
        self.serial = None

class CommandSmoother:
    """Class to smooth movement commands and reduce jitter"""
    
    def __init__(self, buffer_size=5):
        self.buffer_size = buffer_size
        self.left_speed_history = collections.deque(maxlen=buffer_size)
        self.right_speed_history = collections.deque(maxlen=buffer_size)
        self.last_smoothed_command = 'M0,0'
        
    def add_command(self, command):
        """Add a differential steering command to the history buffer"""
        if command and command[0] == 'M' and ',' in command:
            try:
                # Parse the M command format: M<leftSpeed>,<rightSpeed>
                parts = command[1:].split(',')
                if len(parts) == 2:
                    left_speed = int(parts[0])
                    right_speed = int(parts[1])
                    
                    # Add to history
                    self.left_speed_history.append(left_speed)
                    self.right_speed_history.append(right_speed)
            except (ValueError, IndexError):
                # If parsing fails, don't update history
                pass
    
    def get_smoothed_command(self):
        """Get the smoothed differential command by averaging recent values"""
        if len(self.left_speed_history) > 0 and len(self.right_speed_history) > 0:
            # Calculate smoothed speeds by averaging the history
            smoothed_left = int(sum(self.left_speed_history) / len(self.left_speed_history))
            smoothed_right = int(sum(self.right_speed_history) / len(self.right_speed_history))
            
            # Create the smoothed command
            smoothed_command = f'M{smoothed_left},{smoothed_right}'
            self.last_smoothed_command = smoothed_command
            return smoothed_command
        else:
            # Return last command or stop if no history
            return self.last_smoothed_command



class RobustCamera:
    """A robust implementation for camera handling"""
    
    def __init__(self, source=0, reconnect_interval=5, timeout=20):
        self.source = source
        self.reconnect_interval = reconnect_interval
        self.timeout = timeout
        self.cap = None
        self.current_frame = None
        self.running = True
        self.connected = False
        self.lock = threading.Lock()
        self.last_frame_time = 0
        self.frame_timeout = 10  # seconds
        
        # Try to connect initially
        self._connect()
        
        # Start the background thread to monitor the connection
        self.thread = threading.Thread(target=self._camera_monitor)
        self.thread.daemon = True
        self.thread.start()
    
    def _connect(self):
        """Connect to the camera source"""
        try:
            logger.info(f"Connecting to camera source: {self.source}")
            
            # Release any existing capture
            if self.cap is not None:
                self.cap.release()
                
            # Create a new capture
            self.cap = cv2.VideoCapture(self.source)
            
            if not self.cap.isOpened():
                logger.error(f"Failed to open camera source: {self.source}")
                self.connected = False
                return False
                
            # Set buffer size to 1 to get the most recent frame
            self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
            
            # Read a test frame
            ret, frame = self.cap.read()
            if not ret or frame is None:
                logger.error(f"Failed to read initial frame from camera source: {self.source}")
                self.connected = False
                return False
                
            # Success!
            logger.info(f"Successfully connected to camera. Frame size: {frame.shape[1]}x{frame.shape[0]}")
            with self.lock:
                self.current_frame = frame
                self.last_frame_time = time.time()
            
            self.connected = True
            return True
            
        except Exception as e:
            logger.error(f"Error connecting to camera: {e}")
            self.connected = False
            return False
    
    def _camera_monitor(self):
        """Background thread to monitor camera connection and reconnect if needed"""
        while self.running:
            try:
                # Check if the camera is connected and if we've received frames recently
                if (not self.connected or 
                    (self.last_frame_time > 0 and 
                     time.time() - self.last_frame_time > self.frame_timeout)):
                    
                    logger.warning("Camera connection lost or timed out, attempting to reconnect")
                    self._connect()
                    
                # Short sleep to prevent high CPU usage
                time.sleep(1.0)
                    
            except Exception as e:
                logger.error(f"Error in camera monitor: {e}")
                time.sleep(self.reconnect_interval)
    
    def read(self):
        """Read a frame from the camera (similar to cv2.VideoCapture.read())"""
        if not self.connected or self.cap is None:
            return False, None
            
        try:
            # Read a frame from the camera
            ret, frame = self.cap.read()
            
            # If successful, update the current frame and timestamp
            if ret and frame is not None:
                with self.lock:
                    self.current_frame = frame
                    self.last_frame_time = time.time()
                return True, frame
                
            # If we failed to read a frame, the camera might be disconnected
            logger.warning("Failed to read frame from camera")
            return False, None
            
        except Exception as e:
            logger.error(f"Error reading frame: {e}")
            return False, None
    
    def isOpened(self):
        """Check if camera is open (similar to cv2.VideoCapture.isOpened())"""
        return self.connected and self.cap is not None and self.cap.isOpened()
    
    def release(self):
        """Release resources (similar to cv2.VideoCapture.release())"""
        self.running = False
        
        if self.thread.is_alive():
            self.thread.join(timeout=5)
        
        if self.cap is not None:
            try:
                self.cap.release()
            except:
                pass
        
        self.connected = False
        self.cap = None


class LineFollower:
    def __init__(self, camera_source=0, bluetooth_port=None):
        """
        Initialize the line follower
        
        Args:
            camera_source: Camera source (integer for local camera, string for IP camera)
            bluetooth_port: Serial port for Bluetooth connection
        """
        # Initialize camera
        logger.info(f"Initializing camera with source: {camera_source}")
        self.camera = RobustCamera(source=camera_source)
        
        # Initialize Bluetooth connection
        logger.info("Initializing Bluetooth connection")
        self.robot = BluetoothRobot(port=bluetooth_port)

        # Initialize command smoother
        self.command_smoother = CommandSmoother(buffer_size=5)
        
        # Default window size
        self.window_width = 640
        self.window_height = 480
        
        # Running flag for main loop
        self.running = True

        # Frame rate control
        self.fps_limit = 15  # Target frames per second for processing
        self.last_frame_time = 0
        self.frame_interval = 1.0 / self.fps_limit

        # Manual mode variables
        self.manual_mode = False
        self.last_manual_command = None
        self.last_manual_command_time = 0
        self.manual_command_throttle = 0.1  # Throttle manual commands to reduce jitter
        
        # Create control windows
        cv2.namedWindow('Parameters')
        self.setup_trackbars()

        self.sequence_handler = SequenceHandler(self.robot)

    
    def setup_trackbars(self):
        # Create second control window for avoidance parameters to avoid cluttering
        cv2.namedWindow('Avoidance Parameters')
        
        # Window size parameters
        cv2.createTrackbar('Window Width', 'Parameters', 640, 1280, lambda x: None)
        cv2.createTrackbar('Window Height', 'Parameters', 480, 960, lambda x: None)
        
        # ROI parameters for line following
        cv2.createTrackbar('ROI X', 'Parameters', 200, 640, lambda x: None)
        cv2.createTrackbar('ROI Y', 'Parameters', 270, 480, lambda x: None)
        cv2.createTrackbar('ROI Width', 'Parameters', 400, 640, lambda x: None)
        cv2.createTrackbar('ROI Height', 'Parameters', 30, 200, lambda x: None)
        
        # ROI parameters for traffic light detection
        cv2.createTrackbar('Light ROI X', 'Parameters', 190, 640, lambda x: None)
        cv2.createTrackbar('Light ROI Y', 'Parameters', 180, 480, lambda x: None)
        cv2.createTrackbar('Light ROI Width', 'Parameters', 110, 640, lambda x: None)
        cv2.createTrackbar('Light ROI Height', 'Parameters', 70, 480, lambda x: None)
        
        # ROI parameters for green obstacle detection
        cv2.createTrackbar('Obstacle ROI X', 'Parameters', 190, 640, lambda x: None)
        cv2.createTrackbar('Obstacle ROI Y', 'Parameters', 230, 480, lambda x: None)
        cv2.createTrackbar('Obstacle ROI Width', 'Parameters', 170, 640, lambda x: None)
        cv2.createTrackbar('Obstacle ROI Height', 'Parameters', 50, 480, lambda x: None)
        
        # Green color detection parameters
        cv2.createTrackbar('Green Threshold', 'Parameters', 30, 255, lambda x: None)
        cv2.createTrackbar('Green Min Area', 'Parameters', 100, 5000, lambda x: None)
        
        # Red color detection parameters
        cv2.createTrackbar('Red Threshold', 'Parameters', 150, 255, lambda x: None)
        cv2.createTrackbar('Red Min Area', 'Parameters', 100, 5000, lambda x: None)
        
        # Image processing parameters
        cv2.createTrackbar('Threshold', 'Parameters', 60, 255, lambda x: None)
        cv2.createTrackbar('Erode', 'Parameters', 2, 5, lambda x: None)
        cv2.createTrackbar('Dilate', 'Parameters', 2, 5, lambda x: None)
        
        # Speed control parameters
        cv2.createTrackbar('Speed', 'Parameters', 50, 250, lambda x: None)
        cv2.createTrackbar('Turn Speed', 'Parameters', 10, 250, lambda x: None)
        cv2.createTrackbar('Manual Speed', 'Parameters', 100, 250, lambda x: None)
        
        # Arduino handles avoidance sequence
        cv2.createTrackbar('FPS Limit', 'Parameters', 15, 30, lambda x: None)
        cv2.createTrackbar('Sequence Speed', 'Avoidance Parameters', 200, 250, lambda x: None)


    
    def get_trackbar_values(self):
        return {
            # Window size parameters
            'window_width': cv2.getTrackbarPos('Window Width', 'Parameters'),
            'window_height': cv2.getTrackbarPos('Window Height', 'Parameters'),
            
            # ROI parameters for line following
            'roi_x': cv2.getTrackbarPos('ROI X', 'Parameters'),
            'roi_y': cv2.getTrackbarPos('ROI Y', 'Parameters'),
            'roi_width': cv2.getTrackbarPos('ROI Width', 'Parameters'),
            'roi_height': cv2.getTrackbarPos('ROI Height', 'Parameters'),
            
            # ROI parameters for traffic light detection
            'light_roi_x': cv2.getTrackbarPos('Light ROI X', 'Parameters'),
            'light_roi_y': cv2.getTrackbarPos('Light ROI Y', 'Parameters'),
            'light_roi_width': cv2.getTrackbarPos('Light ROI Width', 'Parameters'),
            'light_roi_height': cv2.getTrackbarPos('Light ROI Height', 'Parameters'),
            
            # ROI parameters for green obstacle detection
            'obstacle_roi_x': cv2.getTrackbarPos('Obstacle ROI X', 'Parameters'),
            'obstacle_roi_y': cv2.getTrackbarPos('Obstacle ROI Y', 'Parameters'),
            'obstacle_roi_width': cv2.getTrackbarPos('Obstacle ROI Width', 'Parameters'),
            'obstacle_roi_height': cv2.getTrackbarPos('Obstacle ROI Height', 'Parameters'),
            
            # Green color detection parameters
            'green_threshold': cv2.getTrackbarPos('Green Threshold', 'Parameters'),
            'green_min_area': cv2.getTrackbarPos('Green Min Area', 'Parameters'),
            
            # Red color detection parameters
            'red_threshold': cv2.getTrackbarPos('Red Threshold', 'Parameters'),
            'red_min_area': cv2.getTrackbarPos('Red Min Area', 'Parameters'),
            
            # Image processing parameters
            'threshold': cv2.getTrackbarPos('Threshold', 'Parameters'),
            'erode': cv2.getTrackbarPos('Erode', 'Parameters'),
            'dilate': cv2.getTrackbarPos('Dilate', 'Parameters'),
            'fps_limit': cv2.getTrackbarPos('FPS Limit', 'Parameters'),
            
            # Speed control parameters
            'speed': cv2.getTrackbarPos('Speed', 'Parameters'),
            'turn_speed': cv2.getTrackbarPos('Turn Speed', 'Parameters'),
            'manual_speed': cv2.getTrackbarPos('Manual Speed', 'Parameters'),

            'sequence_speed': cv2.getTrackbarPos('Sequence Speed', 'Avoidance Parameters')
            

        }
    
    def send_manual_command(self, command, force=False):
        """
        Send a manual command with throttling to reduce jitter
        
        Args:
            command (str): Command to send
            force (bool): Force sending even if throttled
        """
        current_time = time.time()
        
        # Skip if same as last command and not enough time has passed (unless forced)
        if (not force and 
            command == self.last_manual_command and 
            current_time - self.last_manual_command_time < self.manual_command_throttle):
            return False
        
        # Send the command
        result = self.robot.send_command(command, force)
        
        if result:
            self.last_manual_command = command
            self.last_manual_command_time = current_time
            
        return result
    
    def detect_green_obstacle(self, frame, params):
        """Detect green obstacles in the specified ROI"""
        # Get obstacle ROI parameters
        obstacle_roi_x = params['obstacle_roi_x']
        obstacle_roi_y = params['obstacle_roi_y']
        obstacle_roi_width = params['obstacle_roi_width']
        obstacle_roi_height = params['obstacle_roi_height']
        
        # Calculate ROI bounds
        x1 = max(0, obstacle_roi_x - obstacle_roi_width//2)
        x2 = min(frame.shape[1], obstacle_roi_x + obstacle_roi_width//2)
        y1 = max(0, obstacle_roi_y - obstacle_roi_height//2)
        y2 = min(frame.shape[0], obstacle_roi_y + obstacle_roi_height//2)
        
        # Extract ROI
        obstacle_roi = frame[y1:y2, x1:x2]
        if obstacle_roi.size == 0:
            return False, frame
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(obstacle_roi, cv2.COLOR_BGR2HSV)
        
        # Define range for green color
        lower_green = np.array([35, 80, 80])
        upper_green = np.array([85, 255, 255])
        
        # Create mask for green color
        green_mask = cv2.inRange(hsv, lower_green, upper_green)
        
        # Find contours of green regions
        contours, _ = cv2.findContours(green_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw obstacle ROI on frame
        cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 255, 0), 2)
        
        # Check if any green area is large enough
        green_obstacle_detected = False
        for c in contours:
            area = cv2.contourArea(c)
            if area > params['green_min_area']:
                green_obstacle_detected = True
                
                # Draw the detected green contour
                cv2.drawContours(obstacle_roi, [c], -1, (255, 255, 0), 2)
                
        if green_obstacle_detected:
            cv2.putText(frame, "GREEN OBSTACLE DETECTED", (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
        
        return green_obstacle_detected, frame
    
    def detect_red_light(self, frame, params):
        """Detect red traffic light in the specified ROI"""
        # Get traffic light ROI
        light_roi_x = params['light_roi_x']
        light_roi_y = params['light_roi_y']
        light_roi_width = params['light_roi_width']
        light_roi_height = params['light_roi_height']
        
        # Calculate ROI bounds
        x1 = max(0, light_roi_x - light_roi_width//2)
        x2 = min(frame.shape[1], light_roi_x + light_roi_width//2)
        y1 = max(0, light_roi_y - light_roi_height//2)
        y2 = min(frame.shape[0], light_roi_y + light_roi_height//2)
        
        # Extract ROI
        light_roi = frame[y1:y2, x1:x2]
        if light_roi.size == 0:
            return False, frame
        
        # Convert to HSV for better color detection
        hsv = cv2.cvtColor(light_roi, cv2.COLOR_BGR2HSV)
        
        # Define range for red color (considering HSV color wheel wraps around)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])
        
        # Create masks for red color
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        red_mask = cv2.bitwise_or(mask1, mask2)
        
        # Find contours of red regions
        contours, _ = cv2.findContours(red_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Draw traffic light ROI on frame
        cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 255), 2)
        
        # Check if any red area is large enough
        red_light_detected = False
        for c in contours:
            area = cv2.contourArea(c)
            if area > params['red_min_area']:
                red_light_detected = True
                
                # Draw the detected red contour and label it
                cv2.drawContours(light_roi, [c], -1, (0, 255, 255), 2)
                
        if red_light_detected:
            cv2.putText(frame, "RED LIGHT - STOP", (x1, y1 - 10), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        return red_light_detected, frame

    def process_single_frame(self, frame):
        """Process a single frame for line following and obstacle detection"""
        # Get parameter values from trackbars
        params = self.get_trackbar_values()
        
        # Create processed frame copy
        processed = frame.copy()
        
        # Update window size if it has changed
        if params['window_width'] != self.window_width or params['window_height'] != self.window_height:
            self.window_width = params['window_width']
            self.window_height = params['window_height']
            cv2.resizeWindow('Camera Feed', self.window_width, self.window_height)
        
        # Update FPS limit if it has changed
        if params['fps_limit'] != self.fps_limit:
            self.fps_limit = params['fps_limit']
            self.frame_interval = 1.0 / self.fps_limit

         # Display manual/auto mode status
        mode_status = "MANUAL MODE" if self.manual_mode else "AUTO MODE"
        mode_color = (0, 0, 255) if self.manual_mode else (0, 255, 0)
        cv2.putText(processed, mode_status, (processed.shape[1] - 200, 30),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
        
        # If in manual mode, skip normal line following processing
        if self.manual_mode:
            cv2.putText(processed, "Manual Control Active", (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
            # Display last manual command if available
            if self.last_manual_command:
                cv2.putText(processed, f"Command: {self.last_manual_command}", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            
            return processed
        
        # Check if obstacle avoidance sequence is running
        if self.sequence_handler.is_running():
            # Update the sequence state
            self.sequence_handler.update()
            
            # Display sequence status
            current_step = self.sequence_handler.current_step + 1
            total_steps = len(self.sequence_handler.sequence_steps)
            left_steps, right_steps, left_speed, right_speed, desc = self.sequence_handler.sequence_steps[self.sequence_handler.current_step]
            
            # Add sequence info to the frame
            cv2.putText(processed, f"Avoidance Sequence: Step {current_step}/{total_steps}", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(processed, f"Action: {desc}", (10, 60),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            cv2.putText(processed, f"L:{left_steps}, R:{right_steps}, Speed:{left_speed}", (10, 90),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            
            # Skip normal line following while sequence is active
            return processed
        
        # Detect traffic light
        red_light_detected, processed = self.detect_red_light(processed, params)
        
        # Detect green obstacle
        green_obstacle_detected, processed = self.detect_green_obstacle(processed, params)
        
        # Handle obstacle avoidance if detected
        if green_obstacle_detected:
            sequence_speed = params['sequence_speed']
            # Start the obstacle avoidance sequence
            self.sequence_handler.start_sequence(speed=sequence_speed)
            cv2.putText(processed, "OBSTACLE: Starting avoidance sequence", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
            return processed
            
        # If red light is detected, stop regardless of line following
        elif red_light_detected:
            # Send stop command using differential steering (both motors at 0)
            self.robot.send_command('M0,0')
            cv2.putText(processed, "Stopped at Red Light", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            return processed
        
        # Get frame dimensions and calculate ROI for line following
        height, width = frame.shape[:2]
        roi_x = params['roi_x']
        roi_y = params['roi_y']
        roi_width = params['roi_width']
        roi_height = params['roi_height']
        
        # Calculate ROI bounds
        x1 = max(0, roi_x - roi_width//2)
        x2 = min(width, roi_x + roi_width//2)
        y1 = max(0, roi_y - roi_height//2)
        y2 = min(height, roi_y + roi_height//2)
        
        # Extract ROI
        roi = frame[y1:y2, x1:x2]
        if roi.size == 0:
            return processed
        
        # Convert ROI to grayscale and blur
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
        blur = cv2.GaussianBlur(gray, (5, 5), 0)
        
        # Threshold
        _, thresh = cv2.threshold(blur, params['threshold'], 255, cv2.THRESH_BINARY_INV)
        
        # Erode and dilate
        mask = cv2.erode(thresh, None, iterations=params['erode'])
        mask = cv2.dilate(mask, None, iterations=params['dilate'])
        
        # Find contours in ROI
        contours, _ = cv2.findContours(mask.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Create debug view
        debug_gray = cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
        debug_thresh = cv2.cvtColor(thresh, cv2.COLOR_GRAY2BGR)
        debug_mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
        
        # Draw ROI boundaries on processed image
        cv2.rectangle(processed, (x1, y1), (x2, y2), (0, 0, 255), 2)
        
        # Draw ROI center lines
        roi_center_x = x1 + roi_width//2
        roi_center_y = y1 + roi_height//2
        cv2.line(processed, (roi_center_x, y1), (roi_center_x, y2), (255, 0, 0), 1)
        cv2.line(processed, (x1, roi_center_y), (x2, roi_center_y), (255, 0, 0), 1)
        
        # Default to stop using differential steering
        left_speed = 0
        right_speed = 0
        status = "No Line"
        
        if len(contours) > 0:
            # Find largest contour
            c = max(contours, key=cv2.contourArea)
            M = cv2.moments(c)
            
            if M['m00'] > 0:
                # Calculate centroid in ROI coordinates
                cx = int(M['m10']/M['m00'])
                cy = int(M['m01']/M['m00'])
                
                # Convert to frame coordinates
                frame_cx = cx + x1
                frame_cy = cy + y1
                
                # Draw centroid
                cv2.circle(processed, (frame_cx, frame_cy), 5, (0, 255, 0), -1)
                cv2.circle(debug_mask, (cx, cy), 5, (0, 255, 0), -1)
                
                # Draw contour on debug view
                cv2.drawContours(debug_mask, [c], -1, (0, 255, 0), 2)
                
                # Calculate offset from center
                offset = cx - roi_width//2
                normalized_offset = offset / (roi_width/2)  # Range: -1 to 1
                
                # Base speed
                base_speed = params['speed']
                
                # Calculate differential steering based on offset
                # The further from center, the more difference between wheel speeds
                # This provides smoother turning than the previous approach
                
                # Scaling factor determines how aggressively we respond to line position
                # Lower values = smoother but slower response, higher values = faster but more jerky response
                scaling_factor = 1.2  # Adjust as needed based on testing
                
                # Turn component: How much to vary from base speed for each wheel
                turn_amount = int(normalized_offset * scaling_factor * base_speed)
                
                # Set wheel speeds with differential steering
                # When line is to the right (positive offset), slow down left wheel
                # When line is to the left (negative offset), slow down right wheel
                left_speed = base_speed + turn_amount
                right_speed = base_speed - turn_amount
                
                # Ensure speeds are within valid range
                left_speed = max(-255, min(255, left_speed))
                right_speed = max(-255, min(255, right_speed))
                
                # Calculate turn severity for display
                turn_severity = abs(normalized_offset)
                
                # Status display
                if turn_severity < 0.2:
                    status = f"Straight: L={left_speed}, R={right_speed}"
                elif normalized_offset > 0:
                    status = f"Right Turn: L={left_speed}, R={right_speed}"
                else:
                    status = f"Left Turn: L={left_speed}, R={right_speed}"
                
                # Draw status
                cv2.putText(processed, status, (10, 30), 
                        cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                cv2.putText(processed, f"Offset: {normalized_offset:.2f}", (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        else:
            # No line detected, stop
            left_speed = 0
            right_speed = 0
            cv2.putText(processed, "No Line Detected", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Send the differential command (using M command for direct motor control)
        differential_command = f'M{left_speed},{right_speed}'
        self.robot.send_command(differential_command)
        
        # Add command info to display
        cv2.putText(processed, f"Cmd: {differential_command}", (10, 90),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
        
        # Create debug composite
        debug_view = np.hstack([debug_gray, debug_thresh, debug_mask])
        
        # Add parameter overlay
        param_text = [
            f"Window: {self.window_width}x{self.window_height}",
            f"FPS Limit: {self.fps_limit}",
            f"Line ROI: {roi_width}x{roi_height}",
            f"Thresh: {params['threshold']}",
            f"Erode: {params['erode']}",
            f"Dilate: {params['dilate']}",
            f"Speed: {params['speed']}",
            f"Manual Speed: {params['manual_speed']}",
            f"Turn Speed: {params['turn_speed']}",
            f"Light ROI: {params['light_roi_width']}x{params['light_roi_height']}",
            f"Red Min Area: {params['red_min_area']}",
            f"Obstacle ROI: {params['obstacle_roi_width']}x{params['obstacle_roi_height']}",
            f"Green Min Area: {params['green_min_area']}",
            f"Mode: {'Manual' if self.manual_mode else 'Auto'}"
        ]
        
        for i, text in enumerate(param_text):
            cv2.putText(debug_view, text, (10, 20 + i*20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 1)
        
        # Resize debug view proportionally
        debug_width = min(1280, self.window_width * 2)
        debug_height = int(debug_view.shape[0] * (debug_width / debug_view.shape[1]))
        debug_view = cv2.resize(debug_view, (debug_width, debug_height))
        
        cv2.imshow('Debug', debug_view)
        
        return processed
    
    def run(self):
        """Main run loop for the line follower"""
        # Create window with default size
        cv2.namedWindow('Camera Feed', cv2.WINDOW_NORMAL)
        cv2.resizeWindow('Camera Feed', self.window_width, self.window_height)
        
        # Create a default frame for when camera fails
        default_frame = np.zeros((480, 640, 3), dtype=np.uint8)
        cv2.putText(default_frame, "Camera Disconnected", (160, 240),
                cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        # Add robot status
        robot_status = "Bluetooth not connected"
        if self.robot.connected:
            robot_status = f"Bluetooth connected: {self.robot.port}"
        cv2.putText(default_frame, robot_status, (160, 280),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        # Main loop
        while self.running:
            try:
                # Try to read frame
                ret, frame = self.camera.read()
                
                if not ret or frame is None:
                    # Display error frame with updated status
                    updated_frame = default_frame.copy()
                    
                    # Update Bluetooth status
                    if self.robot.connected:
                        robot_status = f"Bluetooth connected: {self.robot.port}"
                    else:
                        robot_status = "Bluetooth not connected"
                        
                    cv2.putText(updated_frame, robot_status, (160, 280),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                    
                    # Add mode status
                    mode_status = "MANUAL MODE" if self.manual_mode else "AUTO MODE"
                    mode_color = (0, 0, 255) if self.manual_mode else (0, 255, 0)
                    cv2.putText(updated_frame, mode_status, (160, 320),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
                    
                    # Display error frame
                    cv2.imshow('Camera Feed', updated_frame)
                    
                    # Send stop command for safety
                    self.robot.send_command('M0,0')
                    
                    # Short delay to prevent CPU overload
                    time.sleep(0.1)
                else:
                    # Process frame
                    processed = self.process_single_frame(frame)
                    
                    # Add robot connection status
                    if self.robot.connected:
                        cv2.putText(processed, f"BT: {self.robot.port}", (10, frame.shape[0] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                    else:
                        cv2.putText(processed, "BT: Not Connected", (10, frame.shape[0] - 10),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
                    
                    # Display result
                    cv2.imshow('Camera Feed', processed)
            
            except Exception as e:
                logger.error(f"Error processing frame: {e}")
                
                # Display error frame
                cv2.imshow('Camera Feed', default_frame)
                
                # Send stop command for safety
                self.robot.send_command('M0,0')
                
                # Short delay to prevent CPU overload
                time.sleep(0.6)
            
            # Handle key presses for control
            key = cv2.waitKey(1) & 0xFF
            
            # Get speeds from parameters
            params = self.get_trackbar_values()
            speed = params["manual_speed"] if self.manual_mode else params["speed"]
            turn_speed = params["turn_speed"]
            
            # Handle mode toggle with 'm' key
            if key == ord('m'):
                self.manual_mode = not self.manual_mode
                logger.info(f"Switched to {'MANUAL' if self.manual_mode else 'AUTO'} mode")
                
                # Stop the robot when changing modes for safety
                self.robot.send_command('M0,0', force=True)
                self.last_manual_command = None
                
                # Also stop any running sequence
                if hasattr(self, 'sequence_handler'):
                    self.sequence_handler.stop()
            
            # Process manual controls if in manual mode
            if self.manual_mode:
                # Commands for manual control using differential steering
                if key == ord('w'):  # Forward
                    self.send_manual_command(f'M{speed},{speed}')
                elif key == ord('s'):  # Backward
                    self.send_manual_command(f'M-{speed},-{speed}')
                elif key == ord('a'):  # Left turn
                    self.send_manual_command(f'M{turn_speed},{speed}')
                elif key == ord('d'):  # Right turn
                    self.send_manual_command(f'M{speed},{turn_speed}')
                elif key == ord(' '):  # Stop
                    self.send_manual_command('M0,0', force=True)
            else:
                # In auto mode, only use these keys
                if key == ord(' '):  # Stop
                    self.robot.send_command('M0,0', force=True)
                    # Stop sequence if it's running
                    if hasattr(self, 'sequence_handler'):
                        self.sequence_handler.stop()
            
            # Common controls for both modes
            if key == ord('g'):  # Grab - keep original command
                self.robot.send_command('G0')
            elif key == ord('r'):  # Start obstacle avoidance sequence
                if hasattr(self, 'sequence_handler'):
                    sequence_speed = params["sequence_speed"]
                    self.sequence_handler.start_sequence(speed=sequence_speed)
            elif key == ord('e'):  # Enable motors - keep original command
                self.robot.send_command('E0')
            elif key == 27:  # ESC to quit
                self.running = False
                
        # Cleanup when done
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources before exiting"""
        logger.info("Cleaning up resources...")
        
        try:
            # Send stop command for safety using differential steering
            self.robot.send_command('M0,0')
            time.sleep(0.1)  # Give time for the command to be sent
        except Exception as e:
            logger.error(f"Error stopping robot: {e}")
            
        try:
            # Disconnect from Bluetooth
            if self.robot:
                self.robot.disconnect()
        except Exception as e:
            logger.error(f"Error disconnecting Bluetooth: {e}")
            
        try:
            # Release camera
            if self.camera:
                self.camera.release()
        except Exception as e:
            logger.error(f"Error releasing camera: {e}")
            
        try:
            # Close OpenCV windows
            cv2.destroyAllWindows()
        except Exception as e:
            logger.error(f"Error closing windows: {e}")
        
        logger.info("Cleanup complete")


def list_bluetooth_ports():
    """Helper function to list available Bluetooth ports"""
    ports = []
    try:
        for port in serial.tools.list_ports.comports():
            ports.append({
                'device': port.device,
                'description': port.description,
                'hardware_id': port.hwid
            })
    except Exception as e:
        logger.error(f"Error listing serial ports: {e}")
        
    return ports


def main():
    """Main function to run the line follower"""
    try:
        logger.info("Starting Line Follower with obstacle avoidance sequence...")
        
        # Camera configuration
        print("\nCamera Configuration:")
        print("--------------------")
        print("1: Local Webcam (index 0)")
        print("2: IP Camera (URL)")
        print("3: Enter custom camera index/URL")
        
        camera_choice = input("\nSelect camera option (1-3, default 1): ") or "1"
        
        camera_source = None
        
        try:
            camera_choice = int(camera_choice)
            if camera_choice == 1:
                # Local webcam
                camera_source = 0
                print("Using local webcam (index 0)")
            elif camera_choice == 2:
                # Default IP Camera - request URL
                camera_source = input("Enter IP camera URL: ")
                print(f"Using IP camera at {camera_source}")
            elif camera_choice == 3:
                # Custom source
                camera_input = input("Enter camera index (number) or URL: ")
                # Try to convert to integer for webcam index
                try:
                    camera_source = int(camera_input)
                    print(f"Using local webcam (index {camera_source})")
                except ValueError:
                    # Not a number, treat as URL
                    camera_source = camera_input
                    print(f"Using camera URL: {camera_source}")
            else:
                # Invalid choice, use default
                print("Invalid choice. Using local webcam (index 0).")
                camera_source = 0
        except ValueError:
            # Invalid input, use default
            print("Invalid input. Using local webcam (index 0).")
            camera_source = 0
        
        # Bluetooth port configuration
        print("\nBluetooth Configuration:")
        print("------------------------")
        print("Available serial ports:")
        
        ports = list_bluetooth_ports()
        if not ports:
            print("No serial ports found!")
            
        for i, port in enumerate(ports):
            print(f"{i+1}: {port['device']} - {port['description']}")
        
        print(f"{len(ports)+1}: Enter custom port")
        
        bt_choice = input(f"\nSelect Bluetooth port (1-{len(ports)+1}): ")
        
        bluetooth_port = None
        try:
            bt_choice = int(bt_choice)
            if 1 <= bt_choice <= len(ports):
                bluetooth_port = ports[bt_choice-1]['device']
                print(f"Using Bluetooth port: {bluetooth_port}")
            elif bt_choice == len(ports)+1:
                bluetooth_port = input("Enter custom Bluetooth port: ")
                print(f"Using custom Bluetooth port: {bluetooth_port}")
            else:
                print("Invalid choice. No Bluetooth port selected.")
        except ValueError:
            print("Invalid input. No Bluetooth port selected.")
        
        # Create LineFollower instance
        follower = LineFollower(camera_source=camera_source, bluetooth_port=bluetooth_port)
        
        # If Bluetooth port was specified, try to connect
        if bluetooth_port:
            print(f"Connecting to Bluetooth on port {bluetooth_port}...")
            if follower.robot.connect():
                print("Successfully connected to Bluetooth!")
            else:
                print("Failed to connect to Bluetooth. Will retry in the loop.")
        
        # Run the main loop
        print("\nStarting line following with obstacle avoidance. Press ESC to exit.")
        print("Manual controls:")
        print("  W: Forward")
        print("  S: Backward")
        print("  A: Left turn")
        print("  D: Right turn")
        print("  Space: Stop (also stops avoidance sequence)")
        print("  G: Toggle grabber")
        print("  Q: Start obstacle avoidance sequence manually")
        print("  E: Enable motors")
        
        follower.run()
    
    except KeyboardInterrupt:
        print("Program interrupted by user.")
    except Exception as e:
        logger.error(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        print("Exiting program.")
        if 'follower' in locals():
            follower.cleanup()


if __name__ == "__main__":
    main()