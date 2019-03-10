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
        peersAvailableArray: []
    }
  },
  methods: {
    addPeerConnectionTable(peerName, peerID) {
      var peersTableElement = document.getElementById("peersTable");
      
      //Adding a row for new peer
      var row = peersTableElement.insertRow(0);

      //Preparing new row for new peer
      var name = row.insertCell(0);
      var id = row.insertCell(1);
      var connectionButton = row.insertCell(2);

      //Write peer info in table
      name.innerHTML = peerName;
      id.innerHTML = peerID;

      //Add peer's new connection button in the row
      var newConnectionButton = document.createElement("BUTTON");
      var connectText = document.createTextNode("Connect");
      newConnectionButton.appendChild(connectText);
      var peersTableElement = document.getElementById("peersTable");
      connectionButton.appendChild(newConnectionButton);

      //Add peer infos in array
      peersAvailableArray.push({peerName, peerID});
    },

    removeTopPeerConnectionTable() {
      document.getElementById("peersTable").deleteRow(0);
      peersAvailableArray.shift();
    },

    removeAllPeersConnectionTable() {
      var currentPeer = peersAvailableArray.shift();
      while(currentPeer != undefined){
        removeTopPeerConnectionTable();
      }
    },

/*
    initConnectionComponent(personalId, peersTable) {
      var personalIdElement = document.getElementById("personalId")
      personalIdElement.innerHTML = personalId;

      var connectionTableElement = document.getElementById("connectionTable");
      
      var peersListElement = document.getElementById("peersList");
      addPeerConnectionTable(peerInfos);
    },

    processConnectionComponent(){

    },

    connectionComponent(){
      initConnectionComponent();
      processConnectionComponent();
    }, */

    test(){
      var personalIdElement = document.getElementById("personalId")
      personalIdElement.innerHTML = "TEST ID";

      for(var i = 0; i < 10; i++){
        this.addPeerConnectionTable(i, (i+1));
      }
      
      //this.removeTopPeerConnectionTable();
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
