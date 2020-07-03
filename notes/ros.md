## 👾 ROS Essentials

<p align="justify">
Robot Operating System or ROS provides a means of communication with hardware. It also provides a way for different processes to communicate with one another via message passing. ROS features a slick build and package management system called <b>catkin</b>.

+ Interesting <a href="https://youtu.be/Dm7HnQb8n9Y">documentary</a> by Bloomberg.
+ ROS <a href="http://www.willowgarage.com/sites/default/files/icraoss09-ROS.pdf">paper</a>.

Almost every robot in the world uses:
- Sensors: <b>PERCEPTION</b>
- Software: <b>DECISION MAKING</b>
- Motors & Controllers: <b>ACTUATION</b>

ROS breaks down these high level tasks into small unix processes called nodes.
<center><img src="./img/ros_nodes.PNG"></a></center>
</p>
<p align="justify">ROS master is the manager of all the nodes. ROS master maintains a registry of all the active nodes in a system. Each node can use this registry to discover other nodes and form lines of communication. ROS master also holds parameter server which holds parameters and configuration values that are shared among the running nodes. Nodes share information with each other via topics (/topic_name). In order to send a message to a node, the node must <b>publish</b> to that topic. To receive a message on a topic, a node must <b>subscribe</b> to that topic. Each node may simultaneously publish or subscribe. This network of nodes connected by topics is called a pub-sub architecture.

<b>Message Passing:</b>
There are over 200+ different predefined message types:
- <b>Physical Quantities:</b> position, velocity, acceleration, rotation, duration.
- <b>Sensor Readings:</b> laser scans, images, point clouds, inertial measurements

We can define our own message types.
Pub-Sub model is necessary but there are times when a Request-Response model is necessary. ROS provides <b>services</b>! Like topics, services allow the passing of messages between nodes. However, unlike topics, services do not have any publishers or subscribers. Instead, nodes interacts with the services on a 1:1 basis, using request and response messages.<br />
These networks of nodes are known as <b>Compute Graphs</b>! and are important. ROS provides a system called *rqt_graph* for showing the compute graph of a system.

#### Turtle Sim

The tradition of turtles in robotics is alive and well today. In fact, each recent version of ROS has been named after some sort of turtle. In addition to this turtle-centric naming convention, the Open-Source Robotics Foundation also adds a new turtle to *turtle_sim* with each release.



</p>
