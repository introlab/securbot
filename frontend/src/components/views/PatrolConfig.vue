<template>
  <!-- Patrol Planner page -->
  <b-jumbotron
    id="patrol-planner-layout"
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
        <div
          id="config-container"
          class="h-100 w-100 border rounded shadow-sb"
          style="max-height: 100%"
        >
          <h4 class="m-3">
            Patrol Config
          </h4>
        </div>
      </b-col>
      <!-- Map column -->
      <b-col
        md="8"
        class="mh-100"
      >
        <!-- Map -->
        <div class="h-100 w-100 m-auto position-relative">
          <video-box
            :show="true"
            :zoom="mapZoom"
            :video-id="patrolId"
          />
          <waypoint-overlay
            :is-active="true"
            :is-clickable="false"
            :show="isConnected"
            :zoom="mapZoom"
            :map-size="mapSize"
            :list="waypointList"
            :nb-of-waypoint="-1"
            :video-element="patrolElement"
            @newWaypoint="addWaypointToList"
          />
        </div>
        <div
          v-if="isConnected"
          class="position-absolute overlay-container"
        >
          <div
            id="patrol-overlay-button-container"
            class="overlay-button-container"
          >
            <!-- Zoom Map -->
            <b-button
              id="increase-zoom-button"
              squared
              class="overlay-button"
              @click="increaseZoom"
            >
              <font-awesome-icon icon="plus" />
            </b-button>
            <b-tooltip
              target="increase-zoom-button"
              placement="left"
              variant="secondary"
            >
              Increase Map Zoom
            </b-tooltip>
            <!-- Unzoom Map -->
            <b-button
              id="decrease-zoom-button"
              squared
              class="overlay-button"
              @click="decreaseZoom"
            >
              <font-awesome-icon icon="minus" />
            </b-button>
            <b-tooltip
              target="decrease-zoom-button"
              placement="left"
              variant="secondary"
            >
              Decrease Map Zoom
            </b-tooltip>
          </div>
        </div>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>


<script>
import { mapState } from 'vuex';
import VideoBox from '../widgets/VideoBox';
import SaveLoad from '../widgets/SaveLoad';
import SecurbotTable from '../generic/Table';
import WaypointOverlay from '../generic/WaypointOverlay';

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
 * @since 0.1.0
 * @version 1.0.0
 * @displayName Patrol Planner View
 */
export default {
  name: 'patrol-config',
  components: {
    VideoBox,
    SaveLoad,
    SecurbotTable,
    WaypointOverlay,
  },
  computed: mapState({
    mapZoom: state => state.mapZoom,
    mapSize: state => state.mapSize,
    waypointList: state => state.patrol.waypointList,
    patrolList: state => state.patrol.patrolList,
    headers: state => state.patrol.waypointHeaders,
    patrolId: state => state.htmlElement.patrolId,
    patrolElement: state => state.htmlElement.patrol,
    isConnected: state => state.client.connectionState.robot === 'connected',
  }),
  mounted() {
    this.$store.dispatch('getPatrols');
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    increaseZoom() {
      this.$store.commit('increaseMapZoom');
    },
    decreaseZoom() {
      this.$store.commit('decreaseMapZoom');
    },
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
        this.$store.dispatch('sendPatrol', { patrol: this.waypointList });
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
     * Add a waypoint list to the list.
     *
     * @public
     */
    addWaypointToList(wp) {
      this.$store.commit('addWaypoint', { wp });
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

<style scoped>
.overlay-button {
  background-color: #b5b5b5;
  opacity: 0.4;
  height: 60px !important;
  width: 60px !important;
}
.overlay-button:disabled {
  opacity: 0.2;
  background-color: grey;
}
.overlay-button-container {
  padding: 7px;
  background-color: rgba(245, 245, 245, 0.75);
  /* border: solid;
  border-color: black; */
  border-radius: 5px 0 0 5px;
  margin: auto;
  margin-bottom: 5px;
}
.overlay-container {
  top: 5px;
  right: 15px;
  z-index: 100;
  max-width: 80px;
  height: auto;
}
</style>
