<template>
  <!-- Patrol page -->
  <b-jumbotron
    id="patrol-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100"
    bg-variant="light"
  >
    <!-- Row -->
    <b-row class="h-100">
      <!-- Table column -->
      <b-col
        md="4"
        class="mh-100 d-flex flex-column"
      >
        <!-- Waypoint list -->
        <b-row class="h-50 m-0 mb-1 d-flex flex-column">
          <!-- Waypoint list Container -->
          <div
            class="btn-toolbar mb-1 w-100 d-flex flex-row"
            style="height:40px;"
            role="toolbar"
          >
            <!-- Title -->
            <h4
              class="h-100 text-left"
              style="flex:1;"
            >
              Patrol :
            </h4>
            <!-- Send button -->
            <button
              type="button"
              class="btn btn-success h-100"
              style="align-items:center; width:90px; min-width:80px;"
              @click="sendPatrol()"
            >
              Send
            </button>
            <!-- Clear button -->
            <button
              type="button"
              class="btn btn-danger h-100 ml-1"
              style="align-items:center; width:90px; min-width:80px;"
              @click="clearWaypointList()"
            >
              Reset
            </button>
          </div>
          <div style="height: calc(100% - 40px - 0.25rem)">
            <waypoint-table :waypoint-list="waypointList" />
          </div>
        </b-row>
        <!-- Save-load patrol table -->
        <b-row
          class="m-0 d-flex flex-column"
          style="flex:1"
        >
          <save-load
            :waypoint-list="waypointList"
            :patrol-list="patrolList"
            :bus="bus"
          />
        </b-row>
      </b-col>
      <!-- Map column -->
      <b-col
        md="8"
        class="mh-100"
      >
        <!-- Map -->
        <patrol-map
          :waypoint-list="waypointList"
          patrol-map-id="patrol-map-stream"
        />
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
/**
 * Vue SFC used as a page for patrol planner. This component manages
 * the layout for the patrol planner page. It uses 1 PatrolMap component
 * and 1 WaypointTable component. The PatrolMap shows the map for the robot
 * and allows users to click on the map to set waypoint, those waypoint are
 * then showed in the WaypointTable that allows to remove any undesired points.
 * The page also contains a box with 2 buttons and a title. The buttons are
 * used to send the current waypoint list or clean it (remove every points).
 * It communicates with parent component through the bus in props.
 * This component have the following dependency :
 * PatrolMap.vue Component, WaypointTable.map Component and Bootstrap-Vue
 * for styling.
 *
 * @module Patrol
 * @vue-prop {Vue} bus - Vue bus use to emit event to other components.
 * @vue-prop {Vue} Router - Vue bus use to routing emit event to parent.
 * @vue-event {} destroyed - Event indicating the component has been destroyed.
 * @vue-event {} mounted - Event indicating the component has been mounted.
 * @vue-event {patrol} send-patrol - Event sending the patrol to the layout to send to robot.
 * @vue-event {patrolList} send-patrol-list - Event saving the patrol list on DB.
 * @vue-data {Object[]} waypointList - Lists of the current waypoints.
 * @vue-data {Object[]} patrolList - Lists of saved patrol fetched on DB.
 */

/** Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';
import { mapState } from 'vuex';
import PatrolMap from '../widget/PatrolMap';
import WaypointTable from '../widget/WaypointTable';
import SaveLoad from '../widget/SaveLoad';

export default {
  name: 'patrol-page',
  components: {
    PatrolMap,
    WaypointTable,
    SaveLoad,
  },
  props: {
    bus: {
      type: Vue,
      required: true,
    },
    router: {
      type: Vue,
      required: true,
    },
  },
  computed: mapState({
    waypointList: state => state.patrol.waypointList,
    patrolList: state => state.patrol.patrolList,
  }),
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    console.log('Patrol have been mounted');
    this.router.$emit('mounted');
    this.getSavedPatrols();
  },
  /**
  * Lifecycle Hook - destroyed
  *
  * @method
  * @listens destroyed(el)
  */
  destroyed() {
    console.log('Patrol have been destroyed');
    this.router.$emit('destroyed');
  },
  methods: {
    /**
     * Callback used to get patrols on DB
     * @method
     */
    getSavedPatrols() {
      this.patrolList = JSON.parse('[{"Name":"Test","waypoints":[{"x":593.2924107142857,"y":323.21428571428567,"yaw":0},{"x":550.4352678571429,"y":303.57142857142856,"yaw":0},{"x":518.2924107142858,"y":435.71428571428567,"yaw":0}]}]');
    },
    /**
     * Callback used to send the patrol to the connected robot robot
     * @method
     */
    sendPatrol() {
      const patrolPlan = JSON.stringify({ patrol: this.waypointList, loop: false });

      if (patrolPlan) {
        console.log('Sendig patrolPlan:');
        console.log(patrolPlan);

        this.bus.$emit('send-patrol', patrolPlan);
      }
    },
    sendPatrolList() {
      this.bus.$emit('send-patrol-list', JSON.stringify(this.patrolList));
    },
    /**
     * Callback used to clear the patrol
     * @method
     */
    clearWaypointList() {
      this.$store.commit('clearWaypointList');
    },
    /**
     * Method used to clear the patrol list (delete the list on db)
     * @method
     */
    clearPatrolList() {
      this.$store.commit('clearPatrol');
    },
  },
};
</script>

<style>

</style>
