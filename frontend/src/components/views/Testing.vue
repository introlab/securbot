<template>
  <div
    class="h-100 w-100"
  >
    <h2>Component Testing</h2>
    <div
      style="height:480px; width:704px; margin-top: 25px"
    >
      <interactive-list
        :list="robotList"
        object-key="robotName"
        @click="handleConnection"
      >
        <template v-slot:header>
          <!-- Who Am I ? -->
          <h4
            id="who-am-i"
            class="text-muted ml-1"
          >
            I am : {{ selfId }}
          </h4>
          <!-- Title -->
          <h5 class="ml-1">
            List of Robots:
          </h5>
        </template>
        <template v-slot:tag="slotProp">
          <span
            v-if="slotProp.item.robotId === robotId && connectionState === 'connecting'"
            class="spinner-border spinner-border-sm text-warning"
          />
          <span
            v-else-if="slotProp.item.robotId == robotId"
            class="badge badge-success"
          >
            Connected
          </span>
          <span
            v-else
            class="badge badge-secondary"
          >
            Not Connected
          </span>
        </template>
      </interactive-list>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';
import InteractiveList from '../custom/InteractiveList';

/* global easyrtc */

export default {
  name: 'testing',
  components: {
    InteractiveList,
  },
  data() {
    return {
      vid: null,
    };
  },
  computed: mapState({
    mapStream: state => state.mapStream,
    selfId: state => state.client.myId,
    robotList: state => state.client.robotList,
    robotId: state => state.client.robotId,
    connectionState: state => state.client.connectionState.robot,
    isConnected: state => state.client.connectionState.robot === 'connected',
  }),
  mounted() {
    this.vid = document.getElementById('test-component-videobox');
    this.$nextTick(() => {
      easyrtc.setVideoObjectSrc(this.vid, this.mapStream);
    });
  },
  methods: {
    handleConnection({ robotId }) {
      if (this.isConnected && robotId === this.robotId) {
        console.log('Disconnecting...');
        this.$store.commit('disableJoystick');
        this.$store.dispatch('client/disconnectFromRobot');
      } else if (this.isConnected && this.robotId) {
        console.log('Already connected to someone...');
      } else if (this.connectionState === 'connecting') {
        console.log('Waiting for state...');
      } else {
        this.$store.dispatch('client/connectToRobot', robotId);
      }
    },
    testLogger(item) {
      console.log(item);
      return true;
    },
  },
};
</script>

<style scoped>

</style>
