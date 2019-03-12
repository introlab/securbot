<template>
  <div id="app">
    <h1>SecurBot Vue.js APP</h1>
    <!--
    <joystick/>
    <waypoint/>
    -->
    <div>
      <select v-model="connectionState">
        <option>Connected</option>
        <option>Disconnected</option>
        <option>Failed</option>
        <option>Lost</option>
      </select>
      <button @click="sendState(connectionState)">State</button>
    </div>
    <div>
      <input v-model="iAm" placeholder="my Id">
    </div>
    <select v-model="testPeerId">
      <option v-for="option in something" v-bind:value="option" v-bind:key="option[0].peerId">
        {{option}}
      </option>
    </select>
    <connection :selfId="iAm" :peersTable="testPeerId" :bus="commBus"/>
    <!--
    <demo-container/>
    -->
  </div>
</template>

<script>
//import Joystick from './components/Joystick.vue'
//import Waypoint from './components/Waypoint.vue'
import Connection from './components/Connection.vue'
//import DemoContainer from './components/VideoComponent/DemoContainer.vue'

import Vue from 'vue'
import { constants } from 'fs';

export default {

  name: 'app',
  data(){
    return{
      something:[[{peerName:"Robot1",peerID:"aogiyudlf"},
                {peerName:"Robot2",peerID:"fqw98rasd"}],
                [{peerName:"Pioneer1",peerID:"aogiyudlf"},
                {peerName:"Pioneer2",peerID:"fqw98rasd"}],
                [{peerName:"Turtlebot1",peerID:"aogiyudlf"},
                {peerName:"Turtlebot2",peerID:"fqw98rasd"}]],
      testPeerId: [],
      commBus: new Vue(),
      connectionState:null,
      iAm:null,
    }
  },
  components: {
    //Joystick,
    //Waypoint,
    Connection,
    //DemoContainer
  },
  methods:{
    log(event){
      console.log(event);
    },
    sendState(state){
      this.commBus.$emit('connection-changed', state.toLowerCase());
      //console.log(state);
    }
  },
  mounted(){
    this.commBus.$on('peer-connection', this.log);
  }
};
</script>

<style>
#app {
  font-family: 'Avenir', Helvetica, Arial, sans-serif;
  -webkit-font-smoothing: antialiased;
  -moz-osx-font-smoothing: grayscale;
  text-align: center;
  color: #2c3e50;
  margin-top: 60px;
}

</style>
