<template>
  <div id="connection">
    <h1>connection</h1>
    <!--img src="/static/logo.png" alt="LOGO"-->
    <div id="connectionDiv">
        <h>My ID : </h>
        <h id="personalId"></h>

        <table id="peersTable">
          <th>Robot</th>
          <th>ID</th>
          <th>Connection</th>
        </table>
    </div>

    <!-- TODO: REMOVE WHEN DONE TESTING-->                                                     
    <button v-on:click="test">TEST</button>

  </div>
</template>

<script>
'use strict';

export default {
  name: 'connection',
  data(){
    return {
        peersInfos: []
    }
  },
  methods: {
    addPeerConnectionTable(peerName, peerID) {
      var peersTableElement = document.getElementById("peersTable");
      
      //Adding a row for new peer
      var row = peersTableElement.insertRow(0);

      //Preparing new row for new peer
      var nameCell = row.insertCell(0);
      var idCell = row.insertCell(1);
      var connectionButtonCell = row.insertCell(2);

      //Write peer info in table
      nameCell.innerHTML = peerName;
      idCell.innerHTML = peerID;

      //Add peer's new connection button in the row
      var newConnectionButtonElement = document.createElement("BUTTON");
      newConnectionButtonElement.setAttribute("id", peerID);

      //TODO : add properly an event click listener to activate handlePeerConnection(peerId)
      newConnectionButtonElement.setAttribute("v-on:click", "handlePeerConnection(peerId)");

      var connectText = document.createTextNode("Connect");
      newConnectionButtonElement.appendChild(connectText);

      var peersTableElement = document.getElementById("peersTable");
      connectionButtonCell.appendChild(newConnectionButtonElement);

      //Add peer infos in array
      this.peersInfos.push({name : peerName, id : peerID, isConnected : false});
    },

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

    removeAllPeersConnectionTable() {
      do{
          var currentPeer = this.removeTopPeerConnectionTable();
      } while(currentPeer != undefined)
    },

    getPeerObjectByName(peerName){
      return this.peersInfos.find(function(peerObject){ return peerObject.name == peerName})
    },

    getPeerObjectById(peerId){
      return this.peersInfos.find(function(peerObject){ return peerObject.id == peerId})
    },

    switchPeerConnectionButtonHtmlState(peerId){
      var peerButtonElement = document.getElementById(peerId)
      if(peerButtonElement.innerHTML == "Connect"){
        peerButtonElement.innerHTML = "Disconnect";
      }
      else if (peerButtonElement.innerHTML == "Disconnect"){
        peerButtonElement.innerHTML = "Connect";
      }
      else
        console.log("Error switchPeerConnectionButtonHtmlState")
    },

    connectToPeer(peerId){
      //Emit event
      this.$emit('peer-connection', peerId);
    },

    disconnectToPeer(peerId){
      //Emit event
      this.$emit('peer-disconnection', peerId);
    },

    handlePeerConnection(peerId){
      var peerObject = this.getPeerObjectById(peerId);
      if (peerObject.isConnected == false){
        this.connectToPeer(peerObject.id);

        //TODO: Add check if connectToPeer succeeded : then change boolean and html
        peerObject.isConnected = true;
        this.switchPeerConnectionButtonHtmlState(peerObject.id);
      }
      else if (peerObject.isConnected == true){
        this.disconnectToPeer(peerObject.id);

        //TODO: Add check if connectToPeer succeeded : then change boolean and html
        peerObject.isConnected = false;
        this.switchPeerConnectionButtonHtmlState(peerObject.id);
      }
      else
        console.log("Error handlePeerConnection")
    },

    test(){
      var personalIdElement = document.getElementById("personalId")
      personalIdElement.innerHTML = "TEST ID";

      for(var i = 0; i < 10; i++){
        this.addPeerConnectionTable(i, (i+1));
      }

      // console.log("peerObject Id : " + 5);
      // console.log("name :"+this.getPeerObjectById(5).name);
      // console.log("isConnected :"+this.getPeerObjectById(5).isConnected);

      // this.removeAllPeersConnectionTable();

      // this.switchPeerConnectionButtonHtmlState(5);
      // this.switchPeerConnectionButtonHtmlState(9);

      //this.handlePeerConnection(5);
    }
  }
}
</script>

<style>
#peersTable {
  font-family: "Trebuchet MS", Arial, Helvetica, sans-serif;
  border-collapse: collapse;
  margin-left: auto;
  margin-right: auto;
  /* width: 100%; */
}

#peersTable td, #peersTable th {
  border: 1px solid #ddd;
  padding: 8px;
}

#peersTable tr:nth-child(even){background-color: #f2f2f2;}

#peersTable tr:hover {background-color: #ddd;}

#peersTable th {
  padding-top: 12px;
  padding-bottom: 12px;
  text-align: left;
  background-color: #4CAF50;
  color: white;
}
</style>
