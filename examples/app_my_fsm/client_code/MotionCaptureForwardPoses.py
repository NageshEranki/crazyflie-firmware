import time
import signal
import logging
from queue import SimpleQueue
from threading import Thread, Event
from MotionCaptureTools import QualisysClient

qtm_logger = logging.getLogger('qtm_rt')

class QualisysStreamProcessor(Thread):
    """
    Class for processing streamed data from QualisysClient.
    Runs in a separate thread to allow concurrent operations.
    
    Args:
        ip_address (str): IP address of the QTM server
        rigid_bodies (list[str]): List of rigid body names to track
        frequency (int): Desired streaming frequency in Hz
        
    Override the '_process_body_data' method to modify the behavior of the class when data is pulled from the queue.
    """
    
    def __init__(self, ip_address: str, rigid_bodies: list[str], frequency: int):
        super().__init__()
        self.ip_address = ip_address
        self.rigid_bodies = rigid_bodies
        self.frequency = frequency
        self.pose_queue = SimpleQueue()
        self.client = None
        self._stop_event = Event()
        
        # Set up signal handler for Ctrl+C
        # signal.signal(signal.SIGINT, self._signal_handler)

        self.start()
    
    def signal_handler(self, sig, frame):
        """Handle Ctrl+C gracefully"""
        print("\nShutting down...")
        self.stop()
    
    def run(self):
        """Thread's main method that runs the streaming loop"""
        # Create and start the QualisysClient
        # print(f"Connecting to QTM at {self.ip_address}...")
        qtm_logger.info(f"Connecting to QTM at {self.ip_address}...")
        self.client = QualisysClient(
            ip_address=self.ip_address,
            rigid_bodies=self.rigid_bodies,
            pose_queue=self.pose_queue,
            freq_des=self.frequency
        )
        
        # print("Streaming data. Press Ctrl+C to stop...")
        qtm_logger.info("Streaming data. Press Ctrl+C to stop...")
        
        try:
            self._stream_loop()
        except Exception as e:
            # print(f"\nError in streaming thread: {e}")
            qtm_logger.error(f"Error in streaming thread: {e}")
        finally:
            self._cleanup()
    
    def _stream_loop(self):
        """Main streaming loop"""
        while not self._stop_event.is_set():
            # Get the latest pose data from the queue
            if not self.pose_queue.empty():
                pose_data = self.pose_queue.get()
                
                # Check if we got the termination signal
                if pose_data == 'END':
                    break
                
                # Display data for each rigid body
                for body_name in self.rigid_bodies:
                    self._process_body_data(body_name, pose_data)
            
            # Small sleep to prevent CPU overuse
            time.sleep(0.001)
    
    def _process_body_data(self, body_name: str, pose_data: dict):
        """Display data for a single rigid body"""
        body_data = pose_data.get(body_name)
        if body_data:
            if body_data['t'] != -1:  # Check if the body was detected in this frame
                position = body_data['p']
                print(f"\r{body_name} Position (m): x={position[0]:8.3f}, y={position[1]:8.3f}, z={position[2]:8.3f}", end="")
            else:
                print(f"\r{body_name} not found in current frame", end="")
    
    def _cleanup(self):
        """Clean up resources"""
        if self.client:
            self.client.close()
            # print("\nDisconnected from QTM")
            qtm_logger.info("Disconnected from QTM")
    
    def stop(self):
        """Stop the streaming thread gracefully"""
        self._stop_event.set()
        if self.is_alive():
            self.join()

if __name__ == "__main__":

    # User-defined parameters
    QTM_SERVER_ADDRESS = '128.174.245.151'  # Your QTM server IP
    RIGID_BODY_NAME = "cf1"                 # Your rigid body name
    DESIRED_FREQUENCY = 100                 # Desired frequency in Hz

    # Create and start the viewer
    # Currently, this class prints the position of the rigid body to the console.
    viewer = QualisysStreamProcessor(
        ip_address=QTM_SERVER_ADDRESS,
        rigid_bodies=[RIGID_BODY_NAME],
        frequency=DESIRED_FREQUENCY
    )
    
    # Start the streaming thread
    # viewer.start()
    
    try:
        # Main thread can do other work here
        while viewer.is_alive():
            time.sleep(0.1)
    except KeyboardInterrupt:
        viewer.stop()

