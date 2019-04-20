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
        class="mh-100 d-flex flex-column">
        <b-row class="h-50 m-0 mb-1 d-flex flex-column">
          <div
            class="btn-toolbar mb-1 w-100 d-flex flex-row"
            style="height:40px;"
            role="toolbar">
            <h4
              class="h-100 text-left"
              style="flex:1;">
              Patrol :
            </h4>
            <button
              type="button"
              class="btn btn-success h-100"
              style="align-items:center; width:90px; min-width:80px;"
              @click="sendPatrol()">
              Send
            </button>
            <button
              type="button"
              class="btn btn-danger h-100 ml-1"
              style="align-items:center; width:90px; min-width:80px;"
              @click="clearWaypointList()">
              Reset
            </button>
          </div>
          <div style="height: calc(100% - 40px - 0.25rem)">
            <waypoint-table :waypoint-list="waypointList" />
          </div>
        </b-row>
        <b-row
          class="m-0 d-flex flex-column"
          style="flex:1">
          <save-load
            :waypoint-list="waypointList"
            :patrol-list="patrolList"
            :bus="bus" />
        </b-row>
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
/*
* Author(s):  Edouard Legare <edouard.legare@usherbrooke.ca>,
*             Valerie Gauthier <valerie.gauthier4@usherbrooke.ca>
* File :  Patrol.vue
* Desc :  Vue SFC used as a page for patrol planner. This component manages
*         the layout for the patrol planner page. It uses 1 PatrolMap component
*         and 1 WaypointTable component. The PatrolMap shows the map for the robot
*         and allows users to click on the map to set waypoint, those waypoint are
*         then showed in the WaypointTable that allows to remove any undesired points.
*         The page also contains a box with 2 buttons and a title. The buttons are
*         used to send the current waypoint list or clean it (remove every points).
*         It communicates with parent component through the bus in props.
*
* Dependencies :
*       -PatrolMap.vue
*       -WaypointTable.map
        -SaveLoad.vue
*       -Bootstrap-Vue
*
*/
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
  props: ['bus', 'router'],
  data() {
    return {
      waypointList: [],
      patrolList: [],
    };
  },
  mounted() {
    console.log('Patrol have been mounted');
    this.router.$emit('mounted');
    this.getSavedPatrols();
  },
  destroyed() {
    console.log('Patrol have been destroyed');
    this.router.$emit('destroyed');
  },
  methods: {
    getSavedPatrols() {
      this.patrolList = JSON.parse('[{"Name":"Test","waypoints":[{"x":593.2924107142857,"y":323.21428571428567,"yaw":0},{"x":550.4352678571429,"y":303.57142857142856,"yaw":0},{"x":518.2924107142858,"y":435.71428571428567,"yaw":0}]}]');
    },
    // Send the waypoint list for patrol scheduling
    sendPatrol() {
      // For now only send the first waypoint
      const objective = JSON.stringify(this.waypointList[0]);

      if (objective) {
        console.log('Sendig objective:');
        console.log(objective);

        this.bus.$emit('send-patrol', objective);
      }
    },
    sendPatrolList() {
      this.bus.$emit('send-patrol-list', JSON.stringify(this.patrolList));
    },
    // Clear the waypoint list
    clearWaypointList() {
      this.waypointList = [];
    },
    clearPatrolList() {
      this.patrolList = [];
    },
  },
};
</script>

<style>

</style>
