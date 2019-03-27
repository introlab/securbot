<template>
  <div
    id="connection"
    class="mx-auto"
    style="min-width:320px">
    <div
      id="connection-container"
      class="rounded-lg"
      style="border: 1px solid lightgray;">

      <h4
        id="who-am-i"
        class="text-muted ml-1">I am : {{ selfId }}</h4>
      <h5 class="ml-1">List of Robots:</h5>
      <div class="list-group">
        <button
          v-for="peer in peersTable"
          :key="peer.peerID"
          type="button"
          class="list-group-item-action list-group-item d-flex justify-content-between
          align-items-center peer-item"
          @click="handlePeerConnection(peer.peerID)">
          {{ peer.peerName }}
          <span
            v-if="peer.peerID == isConnectedToPeerId && waitingForConnectionState"
            class="spinner-border spinner-border-sm text-warning"/>
          <span
            v-else-if="peer.peerID == isConnectedToPeerId"
            class="badge badge-success">Connected</span>
          <span
            v-else
            class="badge badge-secondary">Not Connected</span>
        </button >
      </div>
    </div>
  </div>
</template>

<script>
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
*             Anthony Parris <anthony.parris@usherbrooke.ca>,
* File :  Connection.vue
* Desc :  Vue SFC used as a widget that shows the element in the array
*         given in props in a html list. All list element are buttons
*         that, when clicked, request a connection/emit an event on the
*         bus (given in props). It manages the different state on the
*         connection and change a badge to give visual feedback to the
*         user on it. The array given should contains the name and the id
*         of the easyRTC peers that are in the connected room. The component
*         calling this one should manage the easyRTC part and the connection,
*         this component only show peers and manage the connection selection.
*
* Dependencies :
*       -Bootstrap-Vue
*
*/

export default {
  name: 'connection',
  props: ['selfId', 'peersTable', 'bus'],
  data() {
    return {
      fields: ['List of robots', ''],
      isConnected: false,
      isConnectedToPeerId: null,
      waitingForConnectionState: false,
      peersInfos: [],
    };
  },
  // On component mounted, get html elements, set bus event
  mounted() {
    this.bus.$on('connection-changed', this.handleConnectionChanged);
  },
  // On component destroyed, not use for now
  destroyed() {},
  methods: {
    // Switch the state (text in innerHTML) of the peer button - NOT USED - TO BE REMOVED
    switchPeerConnectionButtonHtmlState(peerId, state) {
      if (peerId == null) {
        console.warn('The Id is null...');
        return;
      }

      const peerButtonElement = document.getElementById(peerId);
      if (state === 'Connected') {
        peerButtonElement.innerHTML = 'Disconnect';
        peerButtonElement.className = 'btn btn-outline-success';
      } else if (state === 'Disconnected') {
        peerButtonElement.innerHTML = 'Connect';
        peerButtonElement.className = 'btn btn-outline-danger';
      } else { console.log('Error switchPeerConnectionButtonHtmlState'); }
    },
    // Handle the answer of the connection-changed event
    handleConnectionChanged(state) {
      console.log(`Got event for connection : ${state}`);
      switch (state) {
        case 'connected':
          console.log('Connected!');
          this.waitingForConnectionState = false;
          this.isConnected = true;
          // this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Connected");
          break;
        case 'failed':
          // Popup : Connection Failed...
          console.log('Connection failed...');
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        case 'disconnected':
          console.log('Disconnected!');
          this.isConnected = false;
          // this.switchPeerConnectionButtonHtmlState(this.isConnectedToPeerId, "Disconnected");
          this.isConnectedToPeerId = null;
          break;
        case 'lost':
          // This Id might not be available anymore... Might get a error...
          // Popup : The connection was lost...
          console.log('Connection lost...');
          this.isConnectedToPeerId = null;
          this.waitingForConnectionState = false;
          this.isConnectedToPeerId = null;
          break;
        default:
          console.log(`Something Something : ${state}`);
      }
    },
    // Ask to be connected to a peer
    connectToPeer(peerId) {
      // Emit event
      console.log(`Connecting to : ${peerId}`);
      this.isConnectedToPeerId = peerId;
      this.waitingForConnectionState = true;
      this.bus.$emit('peer-connection', peerId);
      console.log('Connecting...');
    },
    // Ask to be disconnect from the current connected peer (should be from and not to in name)
    disconnectToPeer(peerId) {
      // Emit event
      this.bus.$emit('peer-connection', peerId);
    },
    // Button function to handle the connection/disconnection to a peer
    handlePeerConnection(peerId) {
      if (this.isConnected && peerId === this.isConnectedToPeerId) {
        console.log('Disconnecting...');
        this.disconnectToPeer(this.isConnectedToPeerId);
      } else if (this.isConnected) {
        // Add popup message saying : "You are already connected to this.isConnectedToPeerId,
        // disconnect from it to connect to connect to an other robot"
        console.log('Already connected to someone...');
      } else if (this.waitingForConnectionState) {
        // Popup saying : "Currently trying to connect to this.isConnectedToPeerId,
        // please be patient..."
        console.log('Waiting for state...');
      } else {
        this.connectToPeer(peerId);
      }
    },
  },
};
</script>

<style>
/* Size for list item */
.peer-item{
  min-width: 300px;
  min-height: 30px;
}
</style>
