<template>
  <div id="connection">
    <div id="connection-container">
        <h4 id="who-am-i">I am : {{selfId}}</h4>
        <table id="peers-table">
          <thead class="peer-header">
            <th class="peer-cell">List of robot:</th>
            <!--<th>ID</th>-->
            <th class="peer-cell"></th>
          </thead>
          <tbody id="peers-table-body">
            <tr class="peers-table-row" v-for="peer in peersTable" v-bind:key="peer.peerID">
              <td class="peer-cell">{{peer.peerName}}</td>
              <!--<td>{{peer.peerID}}</td>-->
              <td class="peer-cell content-left">
                <button class="peer-btn"
                  v-bind:id="peer.peerID" 
                  v-on:click="handlePeerConnection(peer.peerID)">
                  Connect
                </button>
              </td>
            </tr>
          </tbody>
        </table>
    </div>
  </div>
</template>

<script>
export default {
  name: 'connection',
  props: ["selfId","peersTable", "bus"],
  data(){
    return {
      whoAmIElement: null,
      peersTableElement: null,
      peersTableBodyElement: null,
      isConnected: false,
      isConnectedToPeerId: null,
      waitingForConnectionState: false,
      peersInfos: []
    }
  },
  methods: {
    //NOT USED
    addPeerConnectionTable(peer) {
      //Adding a row for new peer
      var index = this.peersTableBodyElement.rows.length;
      var row = this.peersTableBodyElement.insertRow(index);

      //Preparing new row for new peer
      var nameCell = row.insertCell(0);
      var idCell = row.insertCell(1);
      var connectionButtonCell = row.insertCell(2);

      //Write peer info in table
      nameCell.innerHTML = peer.peerName;
      idCell.innerHTML = peer.peerID;

      //Add peer's new connection button in the row
      var newConnectionButtonElement = document.createElement("button");
      newConnectionButtonElement.innerHTML = "Connect";
      newConnectionButtonElement.id = peer.peerID;

      //TODO : add properly an event click listener to activate handlePeerConnection(peerId)
      newConnectionButtonElement.onclick = function(){this.handlePeerConnection(peer.peerId);}.bind(this);

      connectionButtonCell.appendChild(newConnectionButtonElement);

      //Add peer infos in array
      this.peersInfos.push({name : peer.peerName, id : peer.peerID});
    },
    //NOT USED
    removeTopPeerConnectionTable() {
      var peersTableElement = document.getElementById("peersTable");
      if(peersTableElement.rows.length > 0){
        peersTableElement.deleteRow(0);
        return this.peersInfos.shift();
      }
      else{
        console.log("Warning removeTopPeerConnectionTable : Can't erase more rows ");
        console.log("Warning removeTopPeerConnectionTable : peersTableElement.length is " + peersTableElement.length);
      }
    },
    //NOT USED
    removeAllPeersConnectionTable() {
      do{
          var currentPeer = this.removeTopPeerConnectionTable();
      } while(currentPeer != undefined)
    },
    //NOT USED
    clearPeerTables(){
      this.peersInfos = [];
      this.peersTableBodyElement = "";
      this.setPeerTableHeaders();
    },
    //NOT USED
    getPeerObjectByName(peerName){
      return this.peersInfos.find(function(peerObject){ return peerObject.name == peerName})
    },
    //NOT USED
    getPeerObjectById(peerId){
      return this.peersInfos.find(function(peerObject){ return peerObject.id == peerId})
    },
    //Switch the state (text in innerHTML) of the peer button
    switchPeerConnectionButtonHtmlState(peerId, state){
      if(peerId == null){
        console.warn("The Id is null...");
        return;
      }

      var peerButtonElement = document.getElementById(peerId)
      if(state == "Connected"){
        peerButtonElement.innerHTML = "Disconnect";
      }
      else if (state == "Disconnected"){
        peerButtonElement.innerHTML = "Connect";
      }
      else
        console.log("Error switchPeerConnectionButtonHtmlState")
    },
    //Handle the answer of the connection-changed event
    handleConnectionChanged(state){
      console.log("Got event for connection : " + state);
      switch(state){
        case 'connected':
          console.log('Connected!');
          this.waitingForConnectionState = false;
          this.isConnected = true;
          this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Connected");
          break;
        case 'failed':
          //Popup : Connection Failed...
          console.log('Connection failed...');
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        case 'disconnected':
          console.log('Disconnected!');
          this.isConnected = false;
          this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Disconnected");
          this.isConnectedToPeerId = null;
          break;
        case 'lost':
          //This is Id might not be available anymore... Might get a error...
          //Popup : The connection was lost...
          console.log('Connection lost...');
          this.isConnectedToPeerId = null;
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        default:
          console.log("Something Something : " + state);
      }
    },
    //Ask to be connected to a peer
    connectToPeer(peerId){
      //Emit event
      console.log("Connecting to : " + peerId)
      this.isConnectedToPeerId = peerId;
      this.waitingForConnectionState = true;
      this.bus.$emit('peer-connection', peerId);
      console.log("Connecting...");
    },
    //Ask to be disconnect from the current connected peer (should be from and not to in name)
    disconnectToPeer(peerId){
      //Emit event
      this.bus.$emit('peer-connection', peerId);
    },
    //Button function to handle the connection/disconnection to a peer
    handlePeerConnection(peerId){
      if(this.isConnected && peerId == this.isConnectedToPeerId){
        console.log("Disconnecting...");
        this.disconnectToPeer(this.isConnectedToPeerId);
      }
      else if(this.isConnected){
        //Add popup message saying : "You are already connected to this.isConnectedToPeerId, 
        //disconnect from it to connect to connect to an other robot"
        console.log("Already connected to someone...");
      }
      else if(this.waitingForConnectionState){
        //Popup saying : "Currently trying to connect to this.isConnectedToPeerId,
        //please be patient..."
        console.log("Waiting for state...");
      }
      else{
        this.connectToPeer(peerId);
      }
    },
  },
  mounted(){
    //Get HTML elements
    this.whoAmIElement = document.getElementById("whoAmI")
    this.peersTableElement = document.getElementById("peersTable");
    this.peersTableBodyElement = document.getElementById("peersTableBody");

    this.bus.$on('connection-changed',this.handleConnectionChanged);
  },
  destroyed(){}
}
</script>

<style>
#connection{
  margin-left: auto;
  margin-right: auto;
}
#who-am-i{
  border-bottom: 1px solid lightgray;
}
#peers-table{
  width:100%;
  border-collapse: collapse;
  /*border: 1px solid #ddd;*/
}
.peers-table-row:nth-child(even) {
  background-color: #f2f2f2;
}
/*
.peers-table-row:hover {
  background-color: rgb(177, 177, 177);
}
*/
.peer-header{
  /*background-color: #4CAF50;
  color: white;*/
  margin-bottom: 10px;
}
.peer-btn{
  border-radius: 5px;
  border: 1px solid gold;
}
.peer-cell{
  min-width: 150px;
  min-height: 30px;
  padding-top: 5px;
  padding-bottom: 5px;
}
.content-left{
  text-align: right;
}
</style>
