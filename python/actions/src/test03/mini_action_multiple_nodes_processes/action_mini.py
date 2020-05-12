import rclpy
import time

from basic_process_classes.action_client_wrapper_process import ActionClientWrapperProcess
from basic_process_classes.action_server_wrapper_process import ActionServerWrapperProcess
   

def main():

    rclpy.init(args=None)
    
    exit_future = rclpy.Future()

    asw = ActionServerWrapperProcess(exit_future)
    asw.start()

    time.sleep(0.5)  # without this instruction this error is thrown at the end of the program: 
    # PyCapsule_GetPointer called with invalid PyCapsule object #3

    acw = ActionClientWrapperProcess(exit_future)
    acw.start()
    
    acw.send_goal()
    acw.join()
    asw.join()
        
    rclpy.shutdown()

if __name__ == '__main__':
    main()