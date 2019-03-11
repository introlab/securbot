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
    <button @click=test()>TEST</button>
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
      newConnectionButtonElement.setAttribute("v-on:click", "handlePeerConnection()");

      var connectText = document.createTextNode("Connect");
      newConnectionButtonElement.appendChild(connectText);

      var peersTableElement = document.getElementById("peersTable");
      connectionButtonCell.appendChild(newConnectionButtonElement);

      //Add peer infos in array
      this.peersInfos.push({name : peerName, id : peerID, isConnected : false});
    },

    removeTopPeerConnectionTable() {
      //TODO : Add a check if already no row before deleting a row
      document.getElementById("peersTable").deleteRow(0);
      return this.peersInfos.shift();
    },

    removeAllPeersConnectionTable() {
      var currentPeer = this.removeTopPeerConnectionTable();
      while(currentPeer != undefined){
        currentPeer = this.removeTopPeerConnectionTable();
      }
    },

    getPeerObjectByName(peerName){
      return peersInfos.find(function(peerObject){ return peerObject.name == peerName})
    },

    getPeerObjectById(peerId){
      return peersInfos.find(function(peerObject){ return peerObject.id == peerId})
    },

    switchPeerConnectionButtonHtmlState(peerId){
      var peerButtonElement = document.getElementById(peerId)
      if(peerButtonElement.innerHTML == "Connect"){
        peerButtonElement.innerHTML = "Disconnect";
        // peerButtonElement.setAttribute("v-on:click", "disconnectToPeer");
      }
      else if (peerButtonElement.innerHTML == "Disconnect"){
        peerButtonElement.innerHTML = "Connect";
        // peerButtonElement.setAttribute("v-on:click", "connectToPeer");
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

    handlePeerConnection(peerObject){
      if (peerObject.isConnected == false){
        connectToPeer(peerObject.id);

        //TODO: Add check if connectToPeer succeeded : then change boolean and html
        peerObject.isConnected = true;
        this.switchPeerConnectionButtonHtmlState(peerObject.id);
      }
      else if (peerObject.isConnected == true){
        disconnectToPeer(peerObject.id);

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
