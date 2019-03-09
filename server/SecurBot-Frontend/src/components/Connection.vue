<template>
  <div id="connection">
    <h1>connection</h1>
    <!--img src="/static/logo.png" alt="LOGO"-->
    <div id="connectionDiv">
        <h>My ID : </h>
        <h id="personalId"></h>

        <table id="peersTable">
          <th>Robots</th>
          <th>IDs</th>
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
    },

  /*   removePeerConnectionTable() {
      document.getElementById("peersList").deleteRow(0);
    },

    initConnectionComponent(personalId, peersList) {
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

      for(var i = 0; i < 10; i++) {
        this.addPeerConnectionTable(i, (i+1));
      }
    }
  }
}


</script>

<style>
table, td {
  border: 1px solid black;
}
</style>
