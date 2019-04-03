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
*       -Bootstrap-Vue
*
*/
import PatrolMap from '../widget/PatrolMap';
import WaypointTable from '../widget/WaypointTable';

export default {
  name: 'patrol-page',
  components: {
    PatrolMap,
    WaypointTable,
  },
  props: ['bus', 'router'],
  data() {
    return {
      waypointList: [],
    };
  },
  mounted() {
    console.log('Patrol have been mounted');
    this.router.$emit('mounted');
  },
  destroyed() {
    console.log('Patrol have been destroyed');
    this.router.$emit('destroyed');
  },
  methods: {
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
    // Clear the waypoint list
    clearWaypointList() {
      this.waypointList = [];
    },
  },
};
</script>

<style>

</style>
