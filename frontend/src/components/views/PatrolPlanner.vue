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
          <div class="d-flex flex-row">
            <h4 class="m-3 w-100">
              Patrol Planner
            </h4>
            <b-button
              variant="success"
              class="mr-3 m-2"
              :disabled="!isConnected"
              @click="sendPatrolToRobot"
            >
              Send
            </b-button>
          </div>
          <div
            id="inner-planner-container"
            class="border rounded mx-1 my-0 p-2"
            style="height: calc( 100% - 60px - 0.25rem );"
          >
            <div
              class="w-100 h-100"
            >
              <div
                class="border rounded m-1 h-100"
              >
                <div
                  class="sb-container-header d-flex flex-row justify-content-between"
                >
                  <h5
                    class="mt-auto"
                    style="max-height: 1.25rem;"
                  >
                    Waypoints
                  </h5>
                  <b-form-select
                    id="patrol-select-input"
                    :value="selectedPatrol"
                    :options="robotPatrol"
                    text-field="name"
                    value-field="info"
                    size="sm"
                    class="sb-selector"
                    @change="loadPatrol"
                  >
                    <template v-slot:first>
                      <option
                        value=""
                      >
                        New Patrol...
                      </option>
                    </template>
                  </b-form-select>
                </div>
                <div
                  class="p-2 h-100"
                >
                  <b-table
                    borderless
                    no-border-collapse
                    sticky-header="calc( 100% - 60px )"
                    :table-class="['m-0', 'table-rounded']"
                    thead-class="text-center"
                    tbody-class="text-center"
                    primary-key="coordinate.x"
                    :fields="headers"
                    :items="waypointList"
                    :tbody-transition-props="transProps"
                  >
                    <template v-slot:cell(index)="data">
                      <div class="d-flex flex-row">
                        <b-button
                          variant="outline-secondary"
                          class="p-0 border-0 mr-1"
                          style="opacity: 0.75"
                          size="sm"
                          @click="increaseWPOrder(data.index)"
                        >
                          <font-awesome-icon icon="chevron-up" />
                        </b-button>
                        {{ data.index + 1 }}
                        <b-button
                          variant="outline-secondary"
                          class="p-0 border-0 ml-1"
                          style="opacity: 0.75"
                          size="sm"
                          @click="decreaseWPOrder(data.index)"
                        >
                          <font-awesome-icon icon="chevron-down" />
                        </b-button>
                      </div>
                    </template>
                    <template v-slot:cell(label)="data">
                      <b-form-input
                        v-model="data.item.label"
                        placeholder="label"
                        style="max-height: 24px; font-size: 0.8rem;"
                        @input="(event) => { $store.commit('setWaypointLabel',
                                                           { index: data.index, value: event }) }"
                        @focus="testFocus"
                      />
                    </template>
                    <template v-slot:cell(holdTime)="data">
                      <b-form-input
                        v-model="data.item.hold_time_s"
                        type="number"
                        min="0"
                        placeholder="sec"
                        style="max-height: 24px; font-size: 0.8rem;"
                        @change="(event) => { $store.commit('setWaypointHold',
                                                            { index: data.index, value: event }) }"
                      />
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
            </div>
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
            :video-id="patrolHTMLId"
          />
          <waypoint-overlay
            :is-active="true"
            :is-clickable="true"
            :show="isConnected"
            :zoom="mapZoom"
            :map-size="mapSize"
            :list="waypointList"
            :nb-of-waypoint="-1"
            :video-element="patrolHTMLElement"
            :refresh-rate="30"
            @newWaypoint="(wp) => { $store.commit('addWaypoint', { wp }) }"
          />
        </div>
        <div
          v-if="isConnected"
          class="position-absolute overlay-container-planner"
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
  data() {
    return {
      transProps: {
        name: 'wp-table',
      },
      selectedRow: '',
    };
  },
  computed: {
    ...mapState({
      currentRobot: state => state.currentRobot,
      mapZoom: state => state.mapZoom,
      mapSize: state => state.mapSize,
      waypointList: state => state.patrol.current.obj.waypoints,
      patrolList: state => state.patrol.list,
      headers: state => state.headers.waypoints,
      currentPatrol: state => state.patrol.current,
      patrolHTMLId: state => state.htmlElement.patrolId,
      patrolHTMLElement: state => state.htmlElement.patrol,
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
    selectedPatrol() {
      if (this.currentPatrol.id) {
        return {
          patrolId: this.currentPatrol.id,
          robotId: this.currentPatrol.obj.robot,
        };
      }
      return '';
    },
  },
  mounted() {
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    testFocus() {
      // console.log(event);
    },
    sendPatrolToRobot() {
      this.$store.dispatch('sendPatrol', { patrol: this.currentPatrol.obj.waypoints });
    },
    fixFloat(value) {
      return value.toFixed(1);
    },
    increaseWPOrder(index) {
      this.$store.commit('reorderWaypoint', { oldIndex: index, newIndex: index - 1 });
    },
    decreaseWPOrder(index) {
      this.$store.commit('reorderWaypoint', { oldIndex: index, newIndex: index + 1 });
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
    clearScheduleData() {
      this.$store.commit('clearCurrentSchedule');
    },
    loadPatrol(event) {
      this.clearScheduleData();
      if (event) {
        this.$store.dispatch('database/getPatrol', event);
      } else {
        this.$store.commit('setCurrentPatrolId', '');
        this.$store.commit('clearCurrentPatrol');
      }
    },
  },
};
</script>

<style>
.wp-table-move {
  transition: transform 1s;
}
.wp-table-enter-active, .wp-table-leave-active {
  transition: all 0.75s;
}
.wp-table-enter, .wp-table-leave-to {
  opacity: 0;
}
.table-b-table-default {
  background-color: #00A759 !important;
  color: white !important;
}
.table-rounded {
  border: solid 1px #00A759;
  border-radius: 0.25rem;
  border-collapse: separate;
}
.table-rounded th {
  border-color: white;
  border-top: none;
}
.table-rounded tr:last-child > td {
  border-bottom: none;
}
.sb-container-header {
  background-color: #00A759;
  color: white;
  padding: 0.5rem;
  border-radius: 0.25rem 0.25rem 0 0;
}
.sb-container-header-title {
  font: 1.25rem;
  font-weight: 500;
  line-height: 1.2;
}
.sb-selector {
  max-width: 150px;
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
.overlay-container-planner {
  top: 5px;
  right: 15px;
  z-index: 100;
  max-width: 80px;
  height: auto;
}
</style>
