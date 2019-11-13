<template>
  <!-- Patrol Config page -->
  <b-jumbotron
    id="patrol-config-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100 bg-transparent"
  >
    <!-- Row -->
    <b-row class="h-100">
      <!-- Table column -->
      <b-col
        md="4"
        class="mh-100"
      >
        <div
          id="planner-container"
          class="h-100 w-100 border rounded shadow-sb"
        >
          <!-- Title -->
          <h4 class="m-3 w-100">
            Patrol Planner
          </h4>
          <div
            id="inner-planner-container"
            class="border rounded mx-1 my-0 p-2"
            style="height: calc( 100% - 60px - 0.25rem );"
          >
            <div
              style="width: 100%; heigth: 60px"
            >
              <b-container fluid>
                <b-row>
                  <b-col
                    sm="6"
                  >
                    <label
                      for="patrol-select-input"
                      class="mt-2"
                    >
                      Load a patrol:
                    </label>
                  </b-col>
                  <b-col
                    sm="6"
                  >
                    <b-form-select
                      id="patrol-select-input"
                      :options="robotPatrol"
                      text-field="name"
                      value-field="info"
                      class="ml-2"
                      @change="loadPatrol"
                    />
                  </b-col>
                </b-row>
              </b-container>
            </div>
            <b-table
              borderless
              striped
              hover
              sticky-header="calc( 100% - 60px )"
              table-class="m-0"
              thead-class="text-center"
              tbody-class="text-center"
              fixed
              :fields="headers"
              :items="waypointList"
            >
              <template v-slot:cell(index)="data">
                {{ data.index + 1 }}
              </template>
              <template v-slot:cell(remove)="data">
                <button
                  :id="'removeBtn'+data.index"
                  type="button"
                  class="btn btn-danger p-0 m-0 border border-secondary h-100 w-75"
                  @click="removeRow(data.index)"
                >
                  <font-awesome-icon icon="trash" />
                </button>
              </template>
            </b-table>
          </div>
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
            :is-clickable="true"
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
import WaypointOverlay from '../generic/WaypointOverlay';

/**
 *
 * Authors:
 *
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>
 * @since 0.1.0
 * @version 1.0.0
 * @displayName Patrol Config View
 */
export default {
  name: 'patrol-planner',
  components: {
    VideoBox,
    WaypointOverlay,
  },
  computed: {
    ...mapState({
      currentRobot: state => state.currentRobot,
      mapZoom: state => state.mapZoom,
      mapSize: state => state.mapSize,
      waypointList: state => state.patrol.waypointList,
      patrolList: state => state.patrol.patrolList,
      headers: state => state.patrol.waypointHeaders,
      patrolId: state => state.htmlElement.patrolId,
      patrolElement: state => state.htmlElement.patrol,
      isConnected: state => state.client.connectionState.robot === 'connected',
    }),
    robotPatrol() {
      const rp = [];
      this.patrolList.forEach((p) => {
        if (p.info.robotId === this.currentRobot.id.db) {
          rp.push(p);
        }
      });
      return rp;
    },
  },
  mounted() {
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    fixFloat(value) {
      return value.toFixed(1);
    },
    increaseZoom() {
      this.$store.commit('increaseMapZoom');
    },
    decreaseZoom() {
      this.$store.commit('decreaseMapZoom');
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
    loadPatrol(event) {
      this.$store.dispatch('database/getPatrol', event);
    },
  },
};
</script>

<style>
.table-b-table-default {
  background-color: #00A759 !important;
  color: white !important;
}
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
