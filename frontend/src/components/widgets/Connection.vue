<template>
  <!-- Connection Widget -->
  <div
    id="connection"
    class="mx-auto"
    style="min-width:320px"
  >
    <!-- Connection Container -->
    <div
      id="connection-container"
      class="rounded-lg"
      style="border: 1px solid lightgray;"
    >
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
      <!-- Container list -->
      <div class="list-group">
        <!-- Create a button per robot in room -->
        <button
          v-for="robot in robotList"
          :key="robot.robotId"
          type="button"
          class="list-group-item-action list-group-item d-flex justify-content-between
          align-items-center peer-item"
          @click="handleConnection(robot.robotId)"
        >
          {{ robot.robotName }}
          <!-- Tag -->
          <span
            v-if="robot.robotId === robotId && connectionState === 'connecting'"
            class="spinner-border spinner-border-sm text-warning"
          />
          <span
            v-else-if="robot.robotId == robotId"
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
        </button>
      </div>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';

/**
 * A component generating a list of button that the operator can click to connect to a robot.
 * NOTE: This componenent will be changed to fit the new /custom/InteractiveList componenent. It
 * will serve the same purpose the layout component serves for the views, aka mostly html and css
 * rules for the underlying/child components.
 *
 * Authors:
 *
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>,
 *    - Anthony Parris - <anthony.parris@usherbrooke.ca>,
 * @since 0.1.0
 * @displayName Connection
 */
export default {
  name: 'connection',
  props: {
    /**
     * The id to display.
     *
     * @public
     */
    selfId: {
      type: String,
      required: true,
    },
    /**
     * The list of robot that be called.
     *
     * @public
     */
    robotList: {
      type: Array,
      required: true,
    },
  },
  computed: mapState('client', {
    isConnected: state => state.connectionState.server,
    robotId: state => state.robotId,
    connectionState: state => state.connectionState.robot,
  }),
  methods: {
    /**
     * Handles the connection changes when a element in the list is clicked.
     *
     * @param {number} robotId The id of the robot to handle.
     * @public
     */
    handleConnection(robotId) {
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
