from ast import While
from concurrent.futures import thread
from xml.dom import minicompat

from matplotlib.pyplot import text
import rclpy 
from rclpy.node import Node
from std_msgs.msg import String
from pathlib import Path
from tkinter import W, Tk, Canvas, Entry, Text, Button, PhotoImage, Label, StringVar
import tkinter.ttk as ttk 
import threading
from queue import Queue


OUTPUT_PATH = Path(__file__).parent
ASSETS_PATH = OUTPUT_PATH / Path("build/assets")


def relative_to_assets(path: str) -> Path:
    return ASSETS_PATH / Path(path)



class GUIHandler(Node, Queue):
    """! Class which inherits from Node class. This class implements
    a tkinter-build interface(in interface method) and a ros node to 
    publish/subscribe to ros topics

    @param Node (class) ROS class to access to ros methods such as:
    create_publisher, create_subscription, etc.
    @param Queue (class) class to pass arguments between threads
    """
 

    def __init__(self):
        """! GUIHandler builder
        """
        super().__init__('GUIHandler')
        Queue.__init__(self)
        self.publisher_ = self.create_publisher(String, 'selection', 10)
        self.subscriber_ = self.create_subscription(String,
                                                    'name_sender',
                                                    lambda msg: self.nameListener(msg),
                                                    10)
                                                    
        #Tkinter handles all events by calling the mainloop method
        #since there are two loops running (ros and tkinter)
        #It is necessary to create a second daemon thread with 
        #tkinter on background
        #self.model_name=""        
        self.model_name = None

        self.x =  threading.Thread(target = self.interface, daemon=True)
        self.x.start()
        #self.x.join()
        

        #This prevents unused variable warnings     
        self.subscriber_



    def messagePublisher(self,event ,message):
        
        """! callback method called when selecting a model class. It 
        publishes the class via ros topic /selection

        @param event (str) event pattern of the form <MODIFIER-MODIFIER-TYPE-DETAIL> 
        @param message (str) class of the model.  It could be either Sidewalk, Oak_tree, etc.
        """
        msg = String()
        msg.data = message
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)   

    def nameListener (self, msg): 
        """! callback that handles ros subscription to topic /name_sender. 
        Gets the actual value of the name to be spawned so that it can be 
        displayed

        @param msg (str) alphanumeric char of the model
        """
        self.model_name = msg.data 
        self.get_logger().info('I heard: "%s"' % self.model_name) 


        #  Please uncomment this if queue approach is considered
        #  consider put() method as a data container
        #  get() method to obtain data and empty the queue

        # try:
        #     self.put(msg.data)
        # except:
        #     print("No succeed")      
  
        
        





         
        



        
    def interface (self):
        """! daemon thread that displays the interface and handles the events
        """
        
        
        window = Tk()

        window.geometry("867x780")
        window.configure(bg = "#F7CA73")

        canvas = Canvas(
        window,
        bg = "#F7CA73",
        height = 780,
        width = 867,
        bd = 0,
        highlightthickness = 0,
        relief = "ridge"
        )

        canvas.place(x = 0, y = 0)
        image_image_1 = PhotoImage(
            file=relative_to_assets("image_1.png"))
        image_1 = canvas.create_image(
            151.0,
            390.0,
            image=image_image_1
        )


        
        
        # ---------------------
        # TEXT
        # ---------------------

        #Set variable text
        my_string_var = StringVar()   
        selectionMode = Label(window,  textvariable = my_string_var, bg="white" ,font=("ubuntu light",12)).place(x=538,y=87) 

        #TODO: Consider queue and thread approach to update
            # the name of the model so it can be displayed on screen
            # create a subscriber to get the response of the /spawn_entity client
            # instantiate a new  StringVar() and set the text to textvariable with success code

            # Uncomment the following lines 
            #model = StringVar() 
            #textMode = Label(window, textvariable = model, bg = "white",font=("ubuntu light",12)).place(x=538,y=100) 

        
      
    

       

       

        #BUTTONS
        button_image_1 = PhotoImage(
            file=relative_to_assets("button_1.png"))
        button_1 = Button(
            image=button_image_1,
            borderwidth=0,
            highlightthickness=0,
            command= lambda: my_string_var.set("Building"),
            relief="flat"
        )

        button_1.place(
            x=460.0,
            y=605.0,
            width=263.0,
            height=62.0
        )

        button_image_2 = PhotoImage(
            file=relative_to_assets("button_2.png"))
        button_2 = Button(
            image=button_image_2,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: my_string_var.set("Wall"),
            relief="flat"
        )
        button_2.place(
            x=460.0,
            y=508.0,
            width=263.0,
            height=62.0
        )
    
        button_image_3 = PhotoImage(
            file=relative_to_assets("button_3.png"))
        button_3 = Button(
            image=button_image_3,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: my_string_var.set("Sidewalk"),
            relief="flat"
        )
        button_3.place(
            x=460.0,
            y=314.0,
            width=263.0,
            height=62.0
        )

        button_image_4 = PhotoImage(
            file=relative_to_assets("button_4.png"))
        button_4 = Button(
            image=button_image_4,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: my_string_var.set("Tree"),
            relief="flat"
        )
        button_4.place(
            x=460.0,
            y=688.0,
            width=263.0,
            height=62.0
        )

        button_image_5 = PhotoImage(
            file=relative_to_assets("button_5.png"))
        button_5 = Button(
            image=button_image_5,
            borderwidth=0,
            highlightthickness=0,
            command=lambda: my_string_var.set("Grass"),           
            relief="flat"
        )
        button_5.place(
            x=460.0,
            y=409.0,
            width=263.0,
            height=62.0
        )

        image_image_2 = PhotoImage(
            file=relative_to_assets("image_2.png"))
        image_2 = canvas.create_image(
            592.0,
            156.0,
            image=image_image_2
        )
        window.resizable(False, False)


        #Click handlers called when a button is pressed
        button_3.bind("<Button-1>", lambda event, arg = 'Sidewalk': self.messagePublisher(event, arg))
        button_5.bind("<Button-1>", lambda event, arg = 'Grass': self.messagePublisher(event, arg))
        button_2.bind("<Button-1>", lambda event, arg = 'Wall': self.messagePublisher(event, arg))
        button_4.bind("<Button-1>", lambda event, arg = 'Oak_tree': self.messagePublisher(event, arg))
        button_1.bind("<Button-1>", lambda event, arg = 'Building': self.messagePublisher(event, arg))

      
        window.mainloop()


def main(args=None):
    
    rclpy.init(args=args)
    minimal_publisher = GUIHandler()
    rclpy.spin(minimal_publisher)




if __name__ == "__main__": 
    main()