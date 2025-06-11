import rclpy  # python client library, allowing connection with ros2
from rclpy.node import Node  # Base class for writing in ros2 node
from nav_msgs.msg import Odometry  # access to Odometry data (position, orientation)
from std_msgs.msg import String
import sqlite3  # import sqlite3 library for the database 
from datetime import datetime  # for time when trash is discovered
import matplotlib.pyplot as plt  # for plotting the trash distribution map
import threading  # for runing node and plot simultaneously
import time  # for the real-time refreshing


class Trashlogger(Node):  # inheritance from node, making it a ros 2 node 
    plasticCount = 0
    paperCount = 0
    canCount = 0
    def __init__(self):
        super().__init__('trash_logger')
        self.subscription = self.create_subscription(
            Odometry,
            '/odom',  # subscribe to odometry to get the location of robot in physical world (x, y, z)
            self.odom_callback,  # everytime robot sends a message... 
            10  # queue size (can change based on how dense the trashs are)
        )

        self.trash_type_subscription = self.create_subscription(
            String,
            '/trash_type',
            self.trash_type_callback,
            10  # queue size (can change based on how dense the trashes are)
        )
        self.conn = sqlite3.connect('trash_log.db', check_same_thread=False)  # open/create sqlite database
        self.lock = threading.Lock() #cus SQL doesn't support mutiple threads reading/writing
        self.create_table()  # creates table if it does not exist 
        self.latest_trash_type = None  # initialize the trash type to be none
        self.clusterWasAcknowledged = False 

    def create_table(self):
        cursor = self.conn.cursor()  # x, y, z need to +5 since robot detects object in 5 cm
        cursor.execute('''
            CREATE TABLE IF NOT EXISTS trash_logs ( 
                timestamp TEXT,
                x REAL, 
                y REAL,
                z REAL,
                trash_type TEXT
            )
        ''')  # create a table called trash_logs with five columns timestamp, (coordinate)x, y, z and trash_type
        self.conn.commit()  # save table to file 

    def trash_type_callback(self, msg): 
        self.latest_trash_type = msg.data
        self.get_logger().info(f'{self.latest_trash_type} has been picked up ')
        # for counting how much trash the robot has on itself (it's relative, does not mach the stuff in the db)
        if self.latest_trash_type == 'plastic':
            Trashlogger.plasticCount += 1
            self.get_logger().info(f'plasticCount: {Trashlogger.plasticCount} ')
        elif self.latest_trash_type == 'paper':
            Trashlogger.paperCount += 1
            self.get_logger().info(f'paperCount: {Trashlogger.paperCount} ')
        elif self.latest_trash_type == 'can':
            Trashlogger.canCount += 1
            self.get_logger().info(f'canCount: {Trashlogger.canCount} ')
        self.storage_full()
    

    def storage_full (self):
        if Trashlogger.plasticCount == 5:
            self.get_logger().info(f'{self.latest_trash_type} has been dropped off')
            Trashlogger.plasticCount = 0
            self.get_logger().info(f'plasticCount: {Trashlogger.plasticCount} ')
        elif Trashlogger.paperCount == 5:
            self.get_logger().info(f'{self.latest_trash_type} has been dropped off')
            Trashlogger.paperCount = 0
            self.get_logger().info(f'paperCount: {Trashlogger.paperCount} ')
        elif Trashlogger.canCount == 5:
            self.get_logger().info(f'{self.latest_trash_type} has been dropped off')
            Trashlogger.canCount = 0
            self.get_logger().info(f'canCount: {Trashlogger.canCount} ')
    
    def check_trash_cluster(self, treshhold = 0.1): #for checking if more robots are needed (more that 5 trash in one cluster)
        if self.clusterWasAcknowledged:
            return
        cursor = self.conn.cursor()
        cursor.execute("SELECT x, y FROM trash_logs")
        data = cursor.fetchall() 

        for i in range(len(data)): #checks the location of all the trash in the db and sees whether more than 5 have the same coordinates (within the range of the treshold (can be adjuested if needed))
            count = 1
            x1, y1 = data[i]
            for j in range(i + 1, len(data)):
                x2, y2 = data[j]
                if abs(x1 - x2) <= treshhold and abs(y1 - y2) <= treshhold:
                    count += 1
            if count >= 5:
                self.get_logger().info(f'more robots needed')
                self.clusterWasAcknowledged = True
                return 
        

    def odom_callback(self, msg):  # runs everytime a new odom message come in 
        if self.latest_trash_type is None:  # if the trash type is not received, then the database do not log 
            return 

        x = msg.pose.pose.position.x + 0.05  # Odometry data (x, y, z)
        y = msg.pose.pose.position.y + 0.05
        z = msg.pose.pose.position.z + 0.05
        timestamp = datetime.now().isoformat()  # the time of trash log yyyy-mm-dd

        with self.lock: #only one thread can run this part at a time, once again because of SQL
            cursor = self.conn.cursor()

            #if self.is_storage_full(cursor):
            #    self.get_logger().warn('storage full, change new trash bag')

            cursor.execute(''' 
                INSERT INTO trash_logs (timestamp, x, y, z, trash_type)
                VALUES (?, ?, ?, ?, ?)
            ''', (timestamp, x, y, z, self.latest_trash_type))
            self.conn.commit()

        self.get_logger().info(f'Logged trash: {self.latest_trash_type} at ({x:.2f}, {y:.2f}, {z:.2f})')
        self.latest_trash_type = None  # reset trash type before the next message
        self.check_trash_cluster()
 

    #def is_storage_full(self, cursor):  # method that checks if the storage is full
    #    cursor.execute('SELECT COUNT (*) FROM trash_logs')  # counts how many rows there are in the trash_logs table
    #    count = cursor.fetchone()[0]  # total number of rows in the table (now)
    #    return count >= 40  # returns true if the rows are more than 40 


def live_plot_loop(db_path='trash_log.db', lock=None):
    plt.ion() #for turning on interactive mode
    fig, ax = plt.subplots()
    colors = {"plastic": "red", "paper": "blue", "can": "green"}

    while True: #runs untill the programm is closed
        time.sleep(0.1) #for updating every 0.1s

        with lock: #only one thread can run this part at a time, once again because of SQL
            conn = sqlite3.connect(db_path, check_same_thread=False)
            cursor = conn.cursor()
            cursor.execute("SELECT x, y, trash_type FROM trash_logs")
            data = cursor.fetchall()
            conn.close() #fetch all of the data from the trash_log db

        if not data:
            continue

        grouped = {}
        for x, y, trash in data: #go through all of the data
            if trash not in grouped: #see if it is a new type of trash
                grouped[trash] = {'x': [], 'y': []} #if yes create an empty list for the coordinates
            grouped[trash]['x'].append(x * 100) #add x coordinates *100 cus it's inniatially in meters
            grouped[trash]['y'].append(y * 100) #add y coordinates *100 cus it's inniatially in meters

        ax.clear()
        for trash, coordinates in grouped.items(): #go throught the group of trash type and its coordinates
            ax.scatter(coordinates['x'], coordinates['y'], color=colors.get(trash, "black"), label=trash)

        ax.set_xlim(0, 75) #set x limit
        ax.set_ylim(0, 75) #set y limit
        ax.set_aspect('equal', adjustable='box') #make plot a square
        ax.set_title("Trash Distribution Map") #set title using uppercase letters
        ax.set_xlabel("x cm") #set x label
        ax.set_ylabel("y cm") #set y label
        ax.grid(True) #show grid
        ax.legend() #create legend
        plt.draw() 
        plt.pause(0.01)#redrawing the plot


def main(args=None):
    rclpy.init(args=args)
    node = Trashlogger()

    plot_thread = threading.Thread(target=live_plot_loop, args=('trash_log.db', node.lock), daemon=True)
    plot_thread.start()
    #creating and starting a thread to make a real-life updates on the trash distribution map

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.conn.close()  # close the node
        node.destroy_node() 
        rclpy.shutdown() 


if __name__ == '__main__':
    main()
