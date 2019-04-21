<template>
  <b-jumbotron
    id="patrol-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100"
    bg-variant="light">
    <b-row class="h-100">
      <b-col
        md="4"
        class="mh-100">
        <div
          class="align-middle p-1"
          style="height:10%">
          <div
            class="w-75 h-100 text-left float-left"
            style="font-size: 20pt">
            Patrol :
          </div>
          <button
            type="button"
            class="btn btn-success w-25 h-50"
            @click="sendPatrol()">
            Confirm
          </button>
          <button
            type="button"
            class="btn btn-danger w-25 h-50"
            @click="clearWaypointList()">
            Reset
          </button>
        </div>
        <div class="h-50">
          <waypoint-table :waypoint-list="waypointList" />
        </div>
        <div />
      </b-col>
      <b-col
        md="8"
        class="mh-100">
        <patrol-map
          :waypoint-list="waypointList"
          patrol-map-id="patrol-map-stream" />
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
 * @vue-data {Object[]} waypointList - Lists the current waypoints
 */

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

import Vue from 'vue';
import PatrolMap from '../widget/PatrolMap';
import WaypointTable from '../widget/WaypointTable';

export default {
  name: 'patrol-page',
  components: {
    PatrolMap,
    WaypointTable,
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
  data() {
    return {
      waypointList: [],
    };
  },
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    console.log('Patrol have been mounted');
    this.bus.$emit('mounted');
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
     * Callback used to send the patrol to scheduling
     * @method
     * @param {Object[]} waypointList - List of current waypoints to be used for patrol
     */
    sendPatrol() {
      this.bus.$emit('send-patrol', JSON.stringify(this.waypointList));
    },
    /**
     * Callback used to clear the patrol
     * @method
     * @param {Object[]} waypointList - List of current waypoints to be used for patrol
     */
    clearWaypointList() {
      this.waypointList = [];
    },
  },
};
</script>

<style>

</style>
