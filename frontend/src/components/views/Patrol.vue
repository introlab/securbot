<template>
  <!-- Patrol page -->
  <b-jumbotron
    id="patrol-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100 bg-transparent"
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
            <!--waypoint-table /-->
            <securbot-table
              :headers="headers"
              :list="waypointList"
              :removable="true"
              @removeRow="removeRow"
            />
          </div>
        </b-row>
        <!-- Save-load patrol table -->
        <b-row
          class="m-0 d-flex flex-column"
          style="flex:1"
        >
          <save-load />
        </b-row>
      </b-col>
      <!-- Map column -->
      <b-col
        md="8"
        class="mh-100"
      >
        <!-- Map -->
        <patrol-map />
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
import { mapState } from 'vuex';
import PatrolMap from '../widgets/PatrolMap';
// import WaypointTable from '../widgets/WaypointTable';
import SaveLoad from '../widgets/SaveLoad';
import SecurbotTable from '../generic/Table';

/**
 * The Patrol Planning Page. This page allows the operator to plan a patrol for the robot. The
 * operator can click the map to add waypoints (with orientation) to the patrol. All waypoints
 * are shown in the tables allowing the operator to remove unwanted waypoints. When a patrol is
 * planned, the operator can save it to the database and/or send it to the robot for it to execute.
 * All the patrols on the databse can be loaded and modified at any time.
 *
 * Authors:
 *
 *    - Valerie Gauthier - <valerie.gauthier@usherbrooke.ca>
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>
 *
 * @version 2.0.0
 * @displayName Patrol Planner View
 */
export default {
  name: 'patrol',
  components: {
    PatrolMap,
    // WaypointTable,
    SaveLoad,
    SecurbotTable,
  },
  computed: mapState({
    waypointList: state => state.patrol.waypointList,
    patrolList: state => state.patrol.patrolList,
    headers: state => state.patrol.waypointHeaders,
  }),
  mounted() {
    this.$store.dispatch('getPatrols');
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    /**
     * Gets the patrol from the database.
     *
     * @public
     */
    getSavedPatrols() {
    },
    /**
     * Sends the patrol to the robot.
     *
     * @public
     */
    sendPatrol() {
      console.log('Sendig patrolPlan:');
      if (this.waypointList.length) {
        this.$store.dispatch('sendPatrol', this.waypointList);
      }
    },
    /**
     * Saves the patrol on the database.
     *
     * @public
     */
    savePatrols() {
      this.$store.dispatch('savePatrols', this.patrolList);
    },
    /**
     * Removes a row from the waypoint list.
     *
     * @public
     */
    removeRow(index) {
      this.$store.commit('removeWaypoint', index);
    },
    /**
     * Removes all waypoints from the patrol.
     *
     * @public
     */
    clearWaypointList() {
      this.$store.commit('clearWaypointList');
    },
    /**
     * Removes all patrols from the server.
     *
     * @public
     */
    clearPatrolList() {
      this.$store.commit('clearPatrol');
    },
  },
};
</script>

<style>

</style>
