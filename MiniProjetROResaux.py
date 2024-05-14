
import networkx as nx
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import numpy as np

# ces libraries ne servent qu'à faire la représentation graphique du résaux 
# Les commentaires sont dans l'espoir d'héberger ce projet sur Github pour le traivailler plus tard pour etre un simulateur de résaux. 
# Les résumés continent les informations nécéssaires à savoir pour lee code de ce projet!  

# A node is the fundamental unit of a graph , we define it by its connections. We add a name just as an identifier for later purposes
class Node:
    """ Un noeud peux être défini par ses voisins. On ajoute juste un nom pour ce noeuds dans l'objectif de l'identifier plus tard  """
    def __init__(self, nodeName):
        self.nodeName = nodeName
        self.adjacents = []

    def setAdjacents(self, adjacents):
        self.adjacents = adjacents



# A graph is an array of Nodes
class Graph:
    """ Une classe pour représenter les graphs. Il s'agit juste d'une liste de noeuds """
    def __init__(self, Nodes, Directed=0):
        self.Nodes = Nodes
        self.Directed = Directed
        self.adjacencyMatrix = None
        if (Directed == 0):
            self.CorrectAdjacents()
    
    
    # corrects nodes Adjacency due to undirected graph
    def CorrectAdjacents(self):
        """ Pour corriger l'adjacence de nos noeuds si le graph n'est pas dirigé puisque certains noeuds peuvent ne pas avoir leur voisins dans leur voisinage """
        for n in self.Nodes:
            for j in self.Nodes:
                if j != n:
                    if j in n.adjacents and n not in j.adjacents:
                        j.adjacents.append(n)

    # defines adjacency Matrix of Graph to use later on for optimisation
    def defAdjacencyMatrix(self):
        """ on établit la matrice d'adjacence en vérifiant tout les noeuds du graph et on regarde si ils sont adjacent avec le neouds courrament traité """
        if self.adjacencyMatrix is None:
            self.ComputeAdjacencyMatrix()
            
    def ComputeAdjacencyMatrix(self):
            if self.Directed:
                self.adjacencyMatrix = [
                    [int(j in i.adjacents) for j in self.Nodes]
                    for i in self.Nodes
                ]
            else:
                self.adjacencyMatrix = [
                    [int(self.Nodes[j] in self.Nodes[i].adjacents) if j >= i else 0 for j in range(len(self.Nodes))]
                    for i in range(len(self.Nodes))
                ]
    def Draw(self):
        """ 
        Représente graphiquement le résaux avec la matrice d'adjacence
        """
        try:
            G = nx.Graph(np.asarray(self.adjacencyMatrix))
        except nx.NetworkXError:
            G=nx.Graph(np.zeros((len(self.Nodes),len(self.Nodes))))
        pos = nx.spring_layout(G)
        labels = {i: self.Nodes[i].nodeName for i in range(len(self.Nodes))}
        for i in range(len(self.Nodes)):
            labels[i]= self.Nodes[i].nodeName
        nx.draw(G, pos, labels=labels, font_weight='bold', node_size=700, node_color='skyblue', font_size=10)
        plt.axis("off")
        plt.show()
        
# Un résaux informatique est un ensemble d'équiepement intermediare (routeur , switch, hub) et finale(pc , telephone , etc etc) on les défini

class EndDevice(Node):
    """ On peux abstraire les périphériques de fin pour le cadre ce projet.
    Les périphériques de fin s'occupent d'envoyer et traiter les packets reçus """
    def __init__(self, nodeName):
        super().__init__(nodeName)
        self.device_type = "End"
        self.AssociatedRouter=None

    def setAdjacents(self, adjacents):
        super().setAdjacents(adjacents)
        
    def FindAssociatedRouter(self):
        """On cherche notre périphérique intermiédaire le plus proche """
        if self.AssociatedRouter:
            return self.AssociatedRouter
        else:
            if self.adjacents:
                for i in self.adjacents:
                    if i.device_type == "Intermediary":
                        self.AssociatedRouter=i
                        return self.AssociatedRouter
            return None
    
    def SendPacket(self,Data,Reciever,Priority="Low"):
        """ 
        Ici on envoi nos données 
        On les encapsule avec notre recepteurs et nous l'envoyeur pour aider à gérer le reste
        Sinon On cherche juste le périphérique intermédiaire le plus proche si le récepteur n'est pas dans notre voisinage
        """
        PacketData=[self,Data,Reciever]
        print(f"{self.nodeName} sends a packet to {Reciever.nodeName}")
        if Reciever in self.adjacents:
            Reciever.RecievePacket(Packet(PacketData,self,Reciever,Priority))
        Routeur=self.FindAssociatedRouter()        
        Routeur.RecievePacket(Packet(PacketData,self,Routeur,Priority))
        
    def RecievePacket(self,RecievedPacket):
        """
        Ici on gére les packets reçu 
        On lit le packet qu'on a reçu. 
        On décapsule le tout et retrouve l'envoyeur original
        Une preuve de reception a été ajoutée juste pour débuger. 
        """
        print(f"{self.nodeName}: {RecievedPacket.Data[0].nodeName} sent you a packet: \n {RecievedPacket.Data[1]}")
        if(RecievedPacket.Data[1][-1] == "SendProofOfReception"):
            self.SendPacket(f"{self.nodeName} has recieved your packet",RecievedPacket.Data[0])

class MidDevice(Node):
    """ Les équipements d'un résaux peuvent être modéliser par des noeuds
    On fait l'abstraction de tout les périphériques intérmédiaire puisque dans le cadre de ce projet leur fonction est presque similaire 
    Cet classe peux:
        librer un packet
        transferer un packet
        filtrer un packet
        Communiquer avec les autres périphérique en cas de problémes
    On utilise comme protocol le Link-state routing protocol , plus d'info sur https://www.ibm.com/docs/en/i/7.4
    """
    
    def __init__(self, nodeName,hasfirewall=0):
        super().__init__(nodeName)
        self.device_type = "Intermediary"
        self.RoutingInfo={}
        self.ShortestRoutingInfo={}
        self.hasFirewall=hasfirewall
        self.hasPacket=0
        self.ConnectedEndDevicesNumber=0
        self.InternalCommunication=EndDevice(self.nodeName+" Communication Handler")
        self.InternalCommunication.AssociatedRouter=self

    def setfirewall(self,blocklist=None,passlist=None):
        self.firewall=Firewall(self.nodeName+" Firewall",blocklist,passlist)
    def filterRecievedPacket(self,RecievedPacket):
        """ Ceci est là où se passe le filtrage de notre packet par le routuer.
        Si le parefeu a une blocklist. Il bloque le packet si il appartient à une source dont on se méfie
        Si c'est une pass list. Le routeur bloque tout les packets avec qui un appareile autre que ceux de confiance on intéragi"""
        firewall=self.firewall 
        if firewall.Passlist:
            for device in RecievedPacket.visited_devices:
                if device not in firewall.Passlist:
                    self.InternalCommunication.SendPacket(f"{device.nodeName} not in the trusted list of {firewall.nodeName}. We have chosen to block it. {RecievedPacket.Data[-1].nodeName} did not recieve your packet ",RecievedPacket.Data[0])
                    return 1
        if firewall.Blocklist:
            for device in RecievedPacket.visited_devices:
                if device in firewall.Blocklist:
                    self.InternalCommunication.SendPacket(f"{device.nodeName} is in the blocklist of {firewall.nodeName}. We have chosen to block it. {RecievedPacket.Data[-1].nodeName} did not recieve your packet ",RecievedPacket.Data[0])
                    return 1            
                
    def countConnectedDevices(self):
        """ Cet méthode sert simplement à conter les périphériques de fin connecter à notre appareile. Celà peux etre utilisé par d'autre méthode"""
        if self.adjacents:
            self.ConnectedEndDevicesNumber=0
            for device in self.adjacents:
                if device.device_type == "End":
                    self.ConnectedEndDevicesNumber=self.ConnectedEndDevicesNumber+1  
        else:
            self.ConnectedEndDevicesNumber=0
                     
    def setAdjacents(self, adjacents):
        super().setAdjacents(adjacents)
        
    def RecievePacket(self,RecievedPacket):
        """ c'est ici que le périphérique s'occupe des packets reçu
        Si il sait où se trouve le destinataire directement il l'envoi au destinataire
        Sinon il cherche sa table de routage
        le filtrage par pare feu et la prioritisation du packet s'effectue ici """
        target_device=RecievedPacket.Data[-1]
        if self.hasFirewall:
            if(self.filterRecievedPacket(RecievedPacket)):
                return 0
        for i in self.adjacents:
            if i== target_device: # Is Reciever directly connected to Device ? go through all devices and tell me
                print(f"{self.nodeName} is directly connected to {target_device.nodeName} , passing it to reciever")
                target_device.RecievePacket(RecievedPacket)
                return 0 # to exit loop
            
        if self not in RecievedPacket.visited_devices:
            RecievedPacket.visited_devices.append(self)
            
        if RecievedPacket.Priority == "High":
            self.HighPriorityTransfer(RecievedPacket)
        else:
            self.TransferPacket(RecievedPacket) # If not , pass it on
    
    def TransferPacket(self, RecievedPacket):
        """ cet fonction transfére le packet reçu. 
        On livre notre packet à la personne qui sait où se trouve le distinataire du packet
        On construit aussi la liste de Routage par flooding suivant le Link-state routing protocol
        Un packet est livré avec ses données.
        Il est encapsulé par ses envoyeurs et recepteurs les plus récents mais les données contienent l'envoyeur et récépteur original. Celà est pour corriger les erreurs de livraison si ces derniéres se produisent
        """
        target_device = RecievedPacket.Data[-1]

        if target_device in self.RoutingInfo:
            next_device = self.RoutingInfo[target_device]
            print(f"{self.nodeName} isn't connected to {target_device.nodeName}, but {next_device.nodeName} knows. Passing it")
            next_device.RecievePacket(RecievedPacket)
        else:
            print(f"{self.nodeName} doesn't know where {target_device.nodeName} is, passing it to all neighbors")
            for neighbor in self.adjacents:
                if neighbor.device_type == "Intermediary" and neighbor not in RecievedPacket.visited_devices:
                    if RecievedPacket.Data[0] not in neighbor.RoutingInfo:
                        neighbor.RoutingInfo[RecievedPacket.Data[0]] = self

                    RecievedPacket.visited_devices.append(neighbor)

                    if target_device in neighbor.adjacents or target_device in neighbor.RoutingInfo:
                        self.RoutingInfo[target_device] = neighbor

                    neighbor.RecievePacket(RecievedPacket)
                    
    # this is a direct translation of the link state Routing protocol that can be found in https://www.ibm.com/docs/en/i/7.4    
    def calculateShortestPaths(self):
        """ Le Link-state routing protocol est divisé en deux parties. Le chemin le plus court et la découvert du graph par flooding
        cet partie est la modification de l'algorithme de Dijkstra pour le Link-state routing protocol
        """
        # Initialize data structures
        done_set = set()
        candidate_list = set()
        distances = {self: 0}
        previous_nodes = {self: None}

        # Step 1: Add the current node to the candidate list
        candidate_list.add(self)

        # Step 2: Run Dijkstra's algorithm
        while candidate_list:
            
            # Find the candidate with the minimum distance
            current_node = min(candidate_list, key=distances.get)
            candidate_list.remove(current_node)
            done_set.add(current_node)

            # Update distances to neighbors
            for neighbor in current_node.adjacents:
                if neighbor not in done_set:
                    cost_to_neighbor = distances[current_node]+1  # Adjust the cost as needed
                    if neighbor not in distances or cost_to_neighbor<distances[neighbor]:
                        distances[neighbor]=cost_to_neighbor
                        previous_nodes[neighbor]=current_node
                        candidate_list.add(neighbor)

        # Step 3: Build routing table
        self.ShortestRoutingInfo = {}
        for node in distances:
            if node!=self:
                current=node
                path=[]
                while current is not None:
                    path.insert(0, current)
                    current = previous_nodes[current]
                self.ShortestRoutingInfo[node] = path  
                  
    def HighPriorityTransfer(self,RecievedPacket):
        """ Cet méthode s'occupe à transferer les packets de haute priorité
        aprés avoir calculer le chemin le plus court , on envoit le packet priortaire sur chemin
        ce chemin le plus court limite le risque d'infiltration et livre le packet rapidement
        si on a une erreur au niveau de la livraison. On passe par un chemin normal
        """
        target_device=RecievedPacket.Data[-1]
        if not(self.ShortestRoutingInfo):
            self.calculateShortestPaths()
            
        if RecievedPacket.Data[-1] in self.ShortestRoutingInfo:
            next_device = self.ShortestRoutingInfo[target_device][1]
            print(f"{self.nodeName} sends the packet meant to {target_device.nodeName} at high priority to {next_device.nodeName}")
            next_device.RecievePacket(RecievedPacket)
            
        else:
            self.InternalCommunication.SendPacket("Dijkstra algorithm failed either due to going through same device multiple times or not enough routing data \n Sending Your packet with normal packets",RecievedPacket.Data[0])
            RecievedPacket.Priority="Low"
            self.TransferPacket(RecievedPacket)            
            


        
    
# A network is a set of computer equipements

class Network(Graph):
    """ 
    Un résaux informatique peux etre générer par un graph ou les sommets sont des équipements.
    Pour les besoins de ce projet. On étuidie simplement la connection en tentant d'optimiser les chémins pris au lieu de se soucier du début
    Autre que représenter un résaux informatique cette classe peux:
        Connecter des périphériques par rapport à une topologie
        Connecter des périphériques automatiquement
        Sécuriser un sous résaux
        
    """

    def __init__(self, Nodes):
        super().__init__(Nodes, 0)
        self.device2_not_found = False # Flag that is used to check if two devices are connected
        
    def Draw(self):
        """ la fonction qui représente graphiquement notre résaux. Adapte simplement le résaux à partir de sa matrice d'adjacence. Rien de particulier Ici"""
        try:
            G = nx.Graph(np.asarray(self.adjacencyMatrix))
        except nx.NetworkXError:
            G=nx.Graph(np.zeros((len(self.Nodes),len(self.Nodes))))
        pos = nx.spring_layout(G)
        labels = {i: self.Nodes[i].nodeName for i in range(len(self.Nodes))}
        colors = {
            "Intermediary": "blue",
            "End": "red",
        }

        node_colors = [colors[node.device_type] for node in self.Nodes]

        for i in range(len(self.Nodes)):
            labels[i] = self.Nodes[i].nodeName
            G.nodes[i]["colors"] = colors[self.Nodes[i].device_type]

        fig, ax = plt.subplots()
        nx.draw(G, pos, labels=labels, font_weight='bold', node_size=700, font_size=10, node_color=node_colors)
        custom_lines = [
            Line2D([0], [0], marker='s', color='w', markerfacecolor='blue', markersize=10),
            Line2D([0], [0], marker='s', color='w', markerfacecolor='red', markersize=10)
        ]

        plt.legend(custom_lines, ['Intermediary Device', 'End Device'], loc='upper right')

        plt.show()
        
    # These functions are here to help optimize and create networks
    def ConnectTopologies(self,Devices,Topology):
        """ 
        Cette méthode implémentes juste quelque topologies déja existantes et communes dans les résaux
        """
        # takes as input a list of devices and a Topology type
        
        # Connects devices in Mesh typology
        if Topology=="Mesh":
            for DeviceToConnect in Devices:
                for device in Devices:
                    if device!=DeviceToConnect:
                        DeviceToConnect.adjacents.append(device)
        # Connects device in star typology
        if Topology=="Star":
            # takes the first device in list as the main device
            for device in Devices:
                if device != Devices[0]:
                    print(f"Connecting {Devices[0].nodeName} to {device.nodeName} in star ")
                    Devices[0].adjacents.append(device)
        
        # Connects the devices in ring typology
        if Topology == "Ring":
            for i in range(len(Devices)-1):
                print(f"Connecting {Devices[i].nodeName} to {Devices[i+1].nodeName} in ring")
                Devices[i].adjacents.append(Devices[i+1])

            # Connect the last device to the first one to complete the ring
            print(f"Connecting {Devices[-1].nodeName} to {Devices[0].nodeName} in ring")
            Devices[-1].adjacents.append(Devices[0])
                
        self.CorrectAdjacents()
        self.ComputeAdjacencyMatrix()
    
    def EstablishSecureNetwork(self,MidDevices):
        """
        Crée le résaux sécurisé à partir d'une liste d'appareille qu'on veux sécuriser
        """
        passlist=MidDevices
        for device in MidDevices:
            if device.device_type == "Intermediary":
                device.hasFirewall = 1
                device.setfirewall([],passlist)
            
    def CheckConnectionBetweenDevices(self, device1, device2, visited_devices=None):
        """ 
        Similaire au concept du flooding
        Cet méthode tente de vérifié si deux périphériques sont connecté directement.
        Sinon on vérifie si les voisins du premier périphériques sont directement connecté au périphérique
        Et on répéte jusqu'à ce qu'il n'y'ai plus de voisins à explorer
        on assume que le périphérique n'est pas dans le résaux mais si on le trouve celà cause toute les autres fonctions à s'arreter en retrant dans une boucle conditionelle qui ne s'active que si l'appareile est trouvé et ne retourne rien mais quitte la fonction
        """
        if visited_devices is None:
            visited_devices = set()
            self.device2_not_found = True  # Assume not found until proven otherwise

        # Check if device2 is found
        if device2 in visited_devices:
            # Commenting it out next line. It only serves if you are debugging and want to see live logs
            # print(f"{device2.nodeName} found. Terminating this call")
            self.device2_not_found = False
            return

        # Mark the current device as visited
        visited_devices.add(device1)

        # Check if device2 is adjacent to device1
        if device2 not in device1.adjacents:
            for neighbor in device1.adjacents:
                if neighbor not in visited_devices:
                    self.CheckConnectionBetweenDevices(neighbor, device2, visited_devices)
                    if not self.device2_not_found:
                        return 1

        # Check if device2 is found in the entire network
        if not self.device2_not_found:
            return 1

        # If device2 is found among neighbors or their neighbors, update the flag
        if device2 in device1.adjacents:
            self.device2_not_found = False
            # Commenting it out next line. It only serves if you are debugging and want to see live logs
            #print(f"{device2.nodeName} found. Terminating this call")
            return 1
            
        if self.device2_not_found:
            # print("device not found")
            return 0
    
    def ConnectTwoDevices(self,device1,device2,DevicesImpossibleToConnectDirectly=[]):
        """Cette methode connecte automatiquement deux périphériques sur le résaux.
        Elle vérifie si l'un d'entre eux est un périphérique intermidaire et si l'autre est un appreille de fin. Elle connecte l'appareille de fin  à Celui du Millieu
        Elle vérifie si deux périphérique du millieu peuvent être directement connecté ils sont directement connecté , sinon on connecte leur voisins
        Si deux périphériques de fins veulent etre connecté on vérifie d'abbord si ils ont un périphérique intermediaire adjcents. Si ils ne l'ont pas , on les attribue au routeurs avec le moins d'élément au moment de la recherche.
            On applique ensuite la connection périphérique intermediaire avec périphérique finale ou  périphérique intermédiaire avec périphérique intermédiire si ils sont tout les deux connecté à un périphérique intermédiaire
        """
        # DevicesImpossibleToConnectDirectly is just an additional contraint to add if we cannot device 1 directly to these devices because the best choice is always just to connect device 1 or its router directly to device 2 or its router
        # print(f"trying to establish connection between {device1.nodeName} and {device2.nodeName} ")
        self.CheckConnectionBetweenDevices(device1,device2)
        if device1 in DevicesImpossibleToConnectDirectly:
            # print("this request doesn't make sense. Resetting the list you gave")
            DevicesImpossibleToConnectDirectly=[]
            
        if not self.device2_not_found:
           print(f"{device2.nodeName} is Already connected to {device1.nodeName}")
        else:
            if (device1.device_type=="End") ^ (device2.device_type=="End"):
                # ^ is a logical XOR. This connect the first device automatically to the other if one of them is a routeur and the other a PC
                if device1.device_type=="End":
                    if device1.AssociatedRouter:
                        device2.adjacents.append(device1.AssociatedRouter)
                        self.CorrectAdjacents()
                        self.ComputeAdjacencyMatrix()
                        return
                    device2.adjacents.append(device1)
                else:
                    if device2.AssociatedRouter:
                        device1.adjacents.append(device2.AssociatedRouter)
                        self.CorrectAdjacents()
                        self.ComputeAdjacencyMatrix()
                        return
                    device1.adjacents.append(device2)
                self.CorrectAdjacents()
                self.ComputeAdjacencyMatrix()
                return
            
            # The next two if loops just correct the devices if they aren't connected to any mid devices if they aren't connected to a router 
            if device1.device_type == "End" and not(device1.AssociatedRouter):
                device1.FindAssociatedRouter()
                if device1.AssociatedRouter is None:
                    m=float("inf")
                    for node in self.Nodes:
                        if node.device_type=="Intermediary" and node not in DevicesImpossibleToConnectDirectly:
                            node.countConnectedDevices()
                            if min(m,node.ConnectedEndDevicesNumber) == node.ConnectedEndDevicesNumber:
                                m=node.ConnectedEndDevicesNumber
                                RouterToConnectDeviceTo=node
                    RouterToConnectDeviceTo.adjacents.append(device1)
                    device1.AssociatedRouter=RouterToConnectDeviceTo
                    self.CorrectAdjacents()


                
            if device2.device_type == "End" and not(device2.AssociatedRouter):
                device2.FindAssociatedRouter()
                if device2.AssociatedRouter is None:
                    m=float("inf")
                    for node in self.Nodes:
                        if node.device_type=="Intermediary" and node not in DevicesImpossibleToConnectDirectly:
                            node.countConnectedDevices()
                            if min(m,node.ConnectedEndDevicesNumber) == node.ConnectedEndDevicesNumber:
                                RouterToConnectDeviceTo=node
                    RouterToConnectDeviceTo.adjacents.append(device2)
                    device2.AssociatedRouter=RouterToConnectDeviceTo
                    self.CorrectAdjacents()

            
            # now we continue our look for potential candidates for connection
            if device1.device_type == "End" and device2.device_type=="Intermediary":
                if device2 not in DevicesImpossibleToConnectDirectly:
                    device2.adjacents.append(device1)
                else:
                    for n in device2.adjacents:
                        self.ConnectTwoDevices(device1,n,DevicesImpossibleToConnectDirectly)
                self.CorrectAdjacents()
                self.ComputeAdjacencyMatrix()
                return
            if device1.device_type == "Intermediary" and device2.device_type=="End":
                if device1 not in DevicesImpossibleToConnectDirectly:
                    device2.AssociatedRouter.adjacents.append(device1)
                else:
                    for n in device2.AssociatedRouter.adjacents:
                        if n.device_type == "Intermediary":
                            self.ConnectTwoDevices(device1,n,DevicesImpossibleToConnectDirectly)
                self.CorrectAdjacents()
                self.ComputeAdjacencyMatrix()
                return
            if device1.device_type=="End" and device2.device_type=="End":
                self.ConnectTwoDevices(device1.AssociatedRouter,device2.AssociatedRouter,DevicesImpossibleToConnectDirectly)
            if device1.device_type == "Intermediary" and device2.device_type=="Intermediary":
                if device1 not in DevicesImpossibleToConnectDirectly and device2 not in DevicesImpossibleToConnectDirectly:
                    device2.adjacents.append(device1)
                else:
                    for n in device2.adjacents:
                        if n.device_type == "Intermediary":
                            self.ConnectTwoDevices(device1,n,DevicesImpossibleToConnectDirectly)
                self.CorrectAdjacents()
                self.ComputeAdjacencyMatrix()
                return
    
    def SmartConnect(self,SecuredNetworksList=[],ContraintDictionnary={}):
        """
        On connecte tout les périphériques du résaux intélligements  avec la méthode connecttwodevices et deux boucles for
        de plus on a un argument optionels pour faire des résaux sécurisé grace au firewall
        Et un dictionaire de contraintes pour chaque périphérique qu'on voudra connecter si on a des contraintes additionelles
        C'est cet partie qui réalise un résaux d'une bonne sécurité et fiabilité 
        """
        # ContraintDictionnary is to be structured like {Device: DevicesImpossibleToConnectDirectly}
        # SecuredNetworksList is the list of all secure networks you want 
        for i in self.Nodes:
            for j in self.Nodes:
                if i != j:
                    if i in ContraintDictionnary:
                        self.ConnectTwoDevices(i,j,ContraintDictionnary[i])
                    else:
                        print(f"we are connecting {i.nodeName} with {j.nodeName}")
                        self.ConnectTwoDevices(i,j)
        
        for SecureNetwork in SecuredNetworksList:
            self.EstablishSecureNetwork(SecureNetwork)
       
                          
# fire wall is what will filter the packets we don't trust       
class Firewall():
    """
    l'object firewall sera attacher au Périphérique intermédiaire.
    Il bloque toute périphérique sur la blocklist et laisse passer que les périphériques sur la passlist
    biensur si l'un est vide. La fonction Bloquer où passer est ingorée
    """
    def __init__(self,nodeName,Blocklist=None,Passlist=None):
        self.nodeName=nodeName
        self.Blocklist=Blocklist
        self.Passlist=Passlist
    
class Packet():
    """
    Le Packet est l'unité du transfer de l'information à travers le résaux.
    Il encapsule l'information en tant q'envoyeur et recepteur
    On a une priorité pour indiquer quel chemin le packet doit prendre
    La visited_devices sert à éviter le looping. Un probléme commun dans la partie flooding Link-state routing protocol. En pratique c'est PacketID mais pour l'étendue académique de ce projet cet solutions suffira
    """
    def __init__(self,Data,Sender,Reciever,Priority="Low"):
        self.Data=Data
        self.Sender=Sender
        self.Reciever=Reciever
        self.Priority=Priority
        self.visited_devices=[]


# Ceci est pour tester les differents types de topologies

PCA=EndDevice("PC A")
PCB=EndDevice("PC B")
PCC=EndDevice("PC C")
PCD=EndDevice("PC D")
PCE=EndDevice("PC E")

TestWiringNetwork=Network([PCA,PCB,PCC,PCD,PCE])

# TestWiringNetwork.ConnectTopologies(TestWiringNetwork.Nodes,"Ring")
TestWiringNetwork.ConnectTopologies(TestWiringNetwork.Nodes,"Star")
# TestWiringNetwork.ConnectTopologies(TestWiringNetwork.Nodes,"Mesh")

TestWiringNetwork.Draw()

# Ceci est pour essayer le cablage Automatique

SmartNetworkPC1=EndDevice("PC1")
SmartNetworkPC2=EndDevice("PC2")
SmartNetworkPC3=EndDevice("PC3")
SmartNetworkPC4=EndDevice("PC4")
SmartNetworkPC5=EndDevice("PC5")
SmartNetworkPC6=EndDevice("PC6")
SmartNetworkPC7=EndDevice("PC7")
SmartNetworkRouter1=MidDevice("Router 1")
SmartNetworkRouter1.setAdjacents([SmartNetworkPC1,SmartNetworkPC2])
SmartNetworkRouter2=MidDevice("Router 2")
SmartNetworkRouter2.setAdjacents([SmartNetworkPC3,SmartNetworkPC4])
SmartNetworkRouter3=MidDevice("Router 3")
SmartNetworkRouter4=MidDevice("Router 4")
SmartNetworkRouter4.setAdjacents([SmartNetworkPC6])
SmartTestNetwork=Network([SmartNetworkPC1,SmartNetworkPC2,SmartNetworkPC3,SmartNetworkPC4,SmartNetworkPC5,SmartNetworkPC6,SmartNetworkPC7,SmartNetworkRouter1,SmartNetworkRouter2,SmartNetworkRouter3,SmartNetworkRouter4])
SmartTestNetwork.defAdjacencyMatrix()
SmartTestNetwork.Draw()
SmartTestNetwork.SmartConnect()
SmartTestNetwork.Draw()



#  Example d'utilisation pour envoyer des paquets


PC1=EndDevice("PC1")
PC2=EndDevice("PC2")
PC3=EndDevice("PC3")
PC4=EndDevice("PC4")
PC5=EndDevice("PC5")
PC6=EndDevice("PC6")
Router1=MidDevice("Routeur 1")
Router2=MidDevice("Routeur 2")
Router3=MidDevice("Routeur 3",1)
Router4=MidDevice("Routeur 4")
Router1.setAdjacents([PC1,PC2])
Router2.setAdjacents([Router1,PC3])
Router3.setAdjacents([Router2,Router1,PC4])
Router4.setAdjacents([Router3,PC5,PC6])
Router3.setfirewall([Router1],[])
network=Network([PC1,PC2,PC3,PC4,PC5,PC6,Router1,Router2,Router3,Router4])
PC1.SendPacket("PC1 Tried to send you a high priority packet",PC6,"High")
PC1.SendPacket(["This is a test packet","SendProofOfReception"],PC6)
PC2.SendPacket(["Hello , this is another test","SendProofOfReception"],PC5)
PC3.SendPacket(["Hello , this is another test","SendProofOfReception"],PC6)
network.defAdjacencyMatrix()
network.Draw()
