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
          <div
            id="inner-patrol-config-container"
            class="border rounded mx-1 my-0 p-2 overflow-auto"
            style="height: calc( 100% - 60px - 0.25rem );"
          >
            <!-- Naming -->
            <div
              class="border rounded m-1 sb-container"
            >
              <div
                class="sb-container-header"
              >
                <h5 class="m-0">
                  <b>Patrol</b>
                </h5>
              </div>
              <div
                class="h-100 p-2"
              >
                <b-form-input
                  id="patrol-name-input"
                  v-model="patrolName"
                  type="text"
                  placeholder="Enter a name..."
                />
              </div>
              <div
                class="h-100 px-2 pb-2 pt-0"
              >
                <b-form-textarea
                  id="patrol-desc-input"
                  v-model="patrolDesc"
                  placeholder="Enter a description..."
                  rows="3"
                  max-rows="3"
                  no-resize
                />
              </div>
            </div>
            <!-- Waypoints -->
            <div
              class="border rounded m-1 sb-container"
            >
              <div
                class="sb-container-header"
              >
                <h5 class="m-0">
                  <b>Waypoints</b>
                </h5>
              </div>
              <div
                class="h-100 p-2"
              >
                <b-form-select
                  id="patrol-name-input"
                  v-model="selectedWaypointIndex"
                  :options="waypointIndexes"
                >
                  <template v-slot:first>
                    <option
                      value=""
                      disabled
                    >
                      Select a Waypoint
                    </option>
                  </template>
                </b-form-select>
              </div>
              <div
                class="h-100 px-2 pb-2 pt-0"
              >
                <b-form-input
                  id="patrol-desc-input"
                  v-model="waypointTimeouts[selectedWaypointIndex]"
                  type="number"
                  placeholder="Enter a time (sec) to hold there..."
                />
              </div>
            </div>
            <!-- Schedule -->
            <div
              class="border rounded m-1 sb-container"
            >
              <div
                class="sb-container-header"
              >
                <h5 class="m-0">
                  <b>Schedule</b>
                </h5>
              </div>
              <div
                class="p-2"
              >
                <b-form-input
                  id="schedule-name-input"
                  v-model="scheduleName"
                  type="text"
                  placeholder="Enter a name..."
                />
              </div>
              <div
                class="px-2 pb-2 pt-0"
              >
                <b-form-textarea
                  id="schedule-desc-input"
                  v-model="scheduleDesc"
                  placeholder="Enter a description..."
                  rows="3"
                  max-rows="3"
                  no-resize
                />
              </div>
              <div
                id="cron-interval-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-select
                  id="cron-interval-input"
                  v-model="cronInterval"
                  :options="cronOptions"
                >
                  <template v-slot:first>
                    <option
                      value=""
                      disabled
                    >
                      Select Desired Occurency
                    </option>
                  </template>
                </b-form-select>
              </div>
              <div
                v-if="!cronInterval[0] && cronInterval.length"
                id="cron-time-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-input
                  id="cron-time-input"
                  v-model="cronTime"
                  type="time"
                />
              </div>
              <div
                v-if="!cronInterval[2] && cronInterval.length"
                id="cron-date-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-input
                  id="cron-time-input"
                  v-model="cronTime"
                  type="date"
                />
              </div>
              <div
                v-if="!cronInterval[0] && cronInterval.length && !cronInterval[4]"
                id="cron-weekday-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-select
                  id="cron-time-input"
                  v-model="cronWeekDay"
                  :options="weekDayOptions"
                />
              </div>
              <div>
                <!-- Repetition + Timeout -->
              </div>
            </div>
            <div
              id="schedule-button-container"
            >
              <b-button
                variant="danger"
                class="float-right m-2"
                @click="clearData"
              >
                Clear
              </b-button>
              <b-button
                variant="success"
                class="float-right m-2"
                :disabled="!isConnected"
                @click="sendToRobot"
              >
                Send
              </b-button>
            </div>
            <b-button
              variant="success"
              class="float-right m-2"
              @click="saveToDB"
            >
              Save
            </b-button>
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
            :is-clickable="false"
            :show="isConnected"
            :zoom="mapZoom"
            :map-size="mapSize"
            :list="waypointList"
            :nb-of-waypoint="-1"
            :video-element="patrolElement"
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
    WaypointOverlay,
  },
  data() {
    return {
      errorSaveToDB: false,
      patrol: {},
      schedule: {},
      patrolName: '',
      patrolDesc: '',
      scheduleName: '',
      scheduleDesc: '',
      scheduleTimeout: '',
      scheduleRepetition: '',
      selectedWaypointIndex: '',
      waypointTimeouts: [],
      cronTime: '',
      cronDate: '',
      cronWeekDay: '',
      cronInterval: '',
      cronOptions: [
        {
          text: 'Once',
          value: ['', '', '', '', ''],
        },
        {
          text: 'Every Hour',
          value: ['', '*', '*', '*', '*'],
        },
        {
          text: 'Every Day',
          value: ['', '', '*', '*', '*'],
        },
        {
          text: 'Every Week',
          value: ['', '', '*', '*', ''],
        },
        {
          text: 'Every Month',
          value: ['', '', '', '*', '*'],
        },
        {
          text: 'Every Year',
          value: ['', '', '', '', '*'],
        },
      ],
      weekDayOptions: [
        {
          text: 'SUNDAY',
          value: 'SUN',
        },
        {
          text: 'MONDAY',
          value: 'MON',
        },
        {
          text: 'TUESDAY',
          value: 'TUE',
        },
        {
          text: 'WEDNESDAY',
          value: 'WED',
        },
        {
          text: 'THURSDAY',
          value: 'THU',
        },
        {
          text: 'FRIDAY',
          value: 'FRI',
        },
        {
          text: 'SATURDAY',
          value: 'SAT',
        },
      ],
    };
  },
  computed: {
    waypointIndexes() {
      const ind = [];
      for (let i = 0; i < this.waypointList.length; i++) {
        ind[i] = {
          text: `${i + 1}`,
          value: i,
        };
      }
      return ind;
    },
    waypointForDB() {
      const wp4db = [];
      const wpList = JSON.parse(JSON.stringify(this.waypointList));
      for (let i = 0; i < wpList.length; i++) {
        wp4db[i] = {
          coordinate: wpList[i],
          hold_time_s: (this.waypointTimeouts[i] ? this.waypointTimeouts[i] : 0),
        };
      }
      return wp4db;
    },
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
  },
  mounted() {
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    increaseZoom() {
      this.$store.commit('increaseMapZoom');
    },
    decreaseZoom() {
      this.$store.commit('decreaseMapZoom');
    },
    clearData() {
      this.patrolName = '';
      this.patrolDesc = '';
      this.scheduleName = '';
      this.scheduleDesc = '';
      this.scheduleTimeout = '';
      this.selectedWaypointIndex = '';
      this.waypointTimeouts = [];
      this.cronTime = '';
      this.cronDate = '';
      this.cronWeekDay = '';
      this.cronInterval = '';
      this.patrol = {};
      this.schedule = {};
    },
    createPlan() {
      if (this.scheduleName) {
        this.createSchedule();
      }
      this.createPatrol();
    },
    createSchedule() {
      const cron = '';
      this.schedule = {
        robot: this.currentRobot.id.db,
        name: this.scheduleName,
        description_text: this.scheduleDesc,
        last_modified: new Date().toISOString(),
        patrol: '',
        cron,
        timeout_s: this.scheduleTimeout,
        repetitions: this.scheduleRepetitions,
        enabled: true,
      };
    },
    createPatrol() {
      this.patrol = {
        robot: this.currentRobot.id.db,
        name: this.patrolName,
        description_text: this.patrolDesc,
        last_modified: new Date().toISOString(),
        waypoints: this.waypointForDB,
      };
    },
    saveToDB() {
      console.log('Sendig patrolPlan:');
      this.createPlan();
      this.$store.dispatch('database/savePatrol', this.patrol)
        .then(() => {
          if (this.scheduleName) {
            this.$store.dispatch('database/saveSchedule', this.schedule);
          }
        }).catch((err) => {
          console.log(err);
          this.errorSaveToDB = true;
          setTimeout(() => {
            this.errorSaveToDB = false;
          }, 2000);
        });
    },
    sendToRobot() {
      this.createPlan();
      this.$store.dispatch('sendPatrol', this.patrol);
    },
  },
};
</script>

<style scoped>
.sb-container-header {
  background-color: #00A759;
  color: white;
  padding: 0.5rem;
  border-radius: 0.25rem 0.25rem 0 0;
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
