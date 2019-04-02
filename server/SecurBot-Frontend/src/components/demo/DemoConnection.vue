<template>
    <div id="demoConnection">
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
    </div>
</template>

<script>
import Connection from '../widget/Connection.vue'

import Vue from 'vue'

export default {
  name: 'demo-connection',
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
    Connection,
  },
  methods:{
    log(event){
      console.log(event);
    },
    sendState(state){
      this.commBus.$emit('connection-changed', state.toLowerCase());
    }
  },
  mounted(){
    this.commBus.$on('peer-connection', this.log);
  }
};
</script>

<style>
</style>