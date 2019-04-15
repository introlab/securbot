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
          class="position-relative"
          style="height:5%; padding:0; margin:2px">
          <div
            class="w-50 h-100 text-left float-left"
            style="font-size: 2.5vh; vertical-align:middle">
            Patrol :
          </div>
          <button
            type="button"
            class="btn btn-success w-25 h-100 float-left"
            style="font-size: 2vmin;align-items:center;vertical-align:middle;
                    margin-left:-2px;padding:0px"
            @click="sendPatrol()">
            Send
          </button>
          <button
            type="button"
            class="btn btn-danger w-25 h-100 float-right"
            style="font-size: 2vmin; align-items:center; position:absolute;
                    padding:0px; margin-left:2px"
            @click="clearWaypointList()">
            Reset
          </button>
        </div>
        <div class="h-50">
          <waypoint-table :waypoint-list="waypointList" />
        </div>
        <div style="height:45%">
          <save-load
            :waypoint-list="waypointList"
            :patrol-list="patrolList"
            :bus="bus" />
        </div>
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
    // Send the waypoint list for patrol execution on the robot
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
