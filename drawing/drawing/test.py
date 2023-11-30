# # import numpy as np

# # Tla = np.array([[0, 1, 0],
# #                 [1, 0, 0],
# #                 [0, 0, -1]])
                
                
# # s = np.sin(-np.pi/3)   
# # c=   np.cos(-np.pi/3)           
# # a =  np.array([[c, 0, s],
# #                 [0, 1, 0],
# #                 [-s, 0, c]])
                
# # print(Tla@a)
                 
# import asyncio

# async def set_after(fut):
#     # Sleep for *delay* seconds.
#     await asyncio.sleep(3)

#     # Set *value* as a result of *fut* Future.
#     fut.set_result(50)

# async def main():
#     # Get the current event loop.
#     loop = asyncio.get_running_loop()

#     # Create a new Future object.
#     fut = loop.create_future()

#     # Run "set_after()" coroutine in a parallel Task.
#     # We are using the low-level "loop.create_task()" API here because
#     # we already have a reference to the event loop at hand.
#     # Otherwise we could have just used "asyncio.create_task()".
#     loop.create_task(
#         set_after(fut))

#     print('hello ...')

#     # Wait until *fut* has a result (1 second) and print it.
#     print(await fut)

# asyncio.run(main())

# import numpy as np  
# T1 = np.array([[ 5.67318741e-01, -8.23498298e-01,  1.66533454e-16,  8.80000000e-02],
#  [ 1.66533454e-16, -2.22044605e-16, -1.00000000e+00, -1.07000000e-01],
#  [ 8.23498298e-01,  5.67318741e-01,  0.00000000e+00,  2.08166817e-17],
#  [ 0.00000000e+00,  0.00000000e+00,  0.00000000e+00,  1.00000000e+00]])

# point = np.array([-0.01, 0, 0.03,1])

# print(T1@point)



import rclpy
from rclpy.node import Node
from std_srvs.srv import Empty
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import asyncio
class Test(Node):

    def __init__(self):
        super().__init__("test")
        self.timer_callback_grp = MutuallyExclusiveCallbackGroup()
        self.calibrate_callback_grp = MutuallyExclusiveCallbackGroup()

        self.timer = self.create_timer(0.1, self.timer_callback, callback_group=self.timer_callback_grp)
        self.calibrate_service = self.create_service(
            Empty, 'calibrate', self.calibrate_callback, callback_group=self.calibrate_callback_grp)
        self.count =0
    #     loop = asyncio.get_running_loop()

    # # Create a new Future object.
    #     fut = loop.create_future()
        # self.future = asyncio.Future()
        self.future = rclpy.task.Future()
        self.state = 0
        
    async def calibrate_callback(self, request, response):
        self.state = 1
        self.get_logger().info('hi in service')
        self.count = 0
        value = await self.future
        self.get_logger().info(f"{value}")

        self.get_logger().info("calibrate")
        return response    
    
    async def timer_callback(self):
        self.get_logger().info('timer running')
        self.count +=1
        if self.state == 1:
            if self.count > 5:
                self.get_logger().info(f'{self.count}')
                self.future.set_result(result= "Done in ros")
            
def Test_entry(args=None):
    rclpy.init(args=args)
    node = Test()
    rclpy.spin(node)
    rclpy.shutdown()

        