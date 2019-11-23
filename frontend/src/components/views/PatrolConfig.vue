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
                class="sb-container-header d-flex flex-row justify-content-between"
              >
                <h5
                  class="mt-auto"
                  style="max-height: 1.25rem;"
                >
                  Patrol
                </h5>
                <b-form-select
                  id="patrol-select-input"
                  v-model="selectedPatrol"
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
                class="p-2"
              >
                <b-form-input
                  id="patrol-name-input"
                  v-model="currentPatrol.obj.name"
                  type="text"
                  placeholder="Enter a name..."
                  @change="(event) => { $store.commit('setCurrentPatrol', { name: event }) }"
                />
              </div>
              <div
                class="px-2 pb-2 pt-0"
              >
                <b-form-textarea
                  id="patrol-desc-input"
                  v-model="currentPatrol.obj.description_text"
                  placeholder="Enter a description..."
                  rows="3"
                  max-rows="3"
                  no-resize
                  @change="(event) =>
                  { $store.commit('setCurrentPatrol', { description_text: event }) }"
                />
              </div>
              <div
                id="patrol-button-container"
                class="d-flex flex-row-reverse"
              >
                <b-button
                  variant="danger"
                  class="mr-2 my-1"
                  :disabled="!currentPatrol.id"
                  @click="deleteCurrentPatrol"
                >
                  Delete
                </b-button>
                <b-button
                  variant="danger"
                  class="mr-2 my-1"
                  @click="clearPatrolData"
                >
                  Clear
                </b-button>
                <b-button
                  variant="success"
                  class="mr-2 my-1"
                  :disabled="!isConnected"
                  @click="sendPatrolToRobot"
                >
                  Send
                </b-button>
                <b-button
                  variant="success"
                  class="mr-2 my-1"
                  :disabled="!isConnected || !currentPatrol.obj.name"
                  @click="savePatrol"
                >
                  Save
                </b-button>
              </div>
            </div>
            <!-- Schedule -->
            <div
              class="border rounded m-1 sb-container"
            >
              <div
                class="sb-container-header d-flex flex-row justify-content-between"
              >
                <h5
                  class="mt-auto"
                  style="max-height: 1.25rem;"
                >
                  Schedule
                </h5>
                <b-form-select
                  id="schedule-select-input"
                  v-model="selectedSchedule"
                  :options="patrolSchedule"
                  text-field="name"
                  value-field="info"
                  size="sm"
                  class="sb-selector"
                  @change="loadSchedule"
                >
                  <template v-slot:first>
                    <option
                      value=""
                    >
                      New Schedule...
                    </option>
                  </template>
                </b-form-select>
              </div>
              <div
                class="p-2"
              >
                <b-form-input
                  id="schedule-name-input"
                  v-model="currentSchedule.obj.name"
                  type="text"
                  placeholder="Enter a name..."
                  @change="(event) => { $store.commit('setCurrentSchedule', { name: event }) }"
                />
              </div>
              <div
                class="px-2 pb-2 pt-0"
              >
                <b-form-textarea
                  id="schedule-desc-input"
                  v-model="currentSchedule.obj.description_text"
                  placeholder="Enter a description..."
                  rows="3"
                  max-rows="3"
                  no-resize
                  @change="(event) =>
                  { $store.commit('setCurrentSchedule', { description_text: event }) }"
                />
              </div>
              <div
                class="px-2 pb-2 pt-0"
              >
                <b-form-input
                  id="schedule-repetition-input"
                  v-model="currentSchedule.obj.repetitions"
                  type="number"
                  placeholder="Enter the number of time to repeat..."
                  @change="(event) =>
                  { $store.commit('setCurrentSchedule', { repetitions: event }) }"
                />
              </div>
              <div
                class="px-2 pb-2 pt-0"
              >
                <b-form-input
                  id="schedule-timeout-input"
                  v-model="currentSchedule.obj.timeout_s"
                  type="number"
                  placeholder="Enter the timeout time (sec)..."
                  @change="(event) => { $store.commit('setCurrentSchedule', { timeout_s: event }) }"
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
                  @change="setCron"
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
                id="cron-time-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-group
                  label-cols-md="4"
                  label="Enter Time:"
                  class="m-0 pl-2"
                >
                  <div
                    class="d-flex justify-content-end"
                  >
                    <b-form-input
                      id="cron-time-hour-input"
                      v-model="cronSchedule[1]"
                      type="number"
                      placeholder="HH"
                      min="0"
                      max="23"
                      style="max-width: 4.5rem"
                      :disabled="cron[1] === '*' || !cronInterval.length"
                      @change="(value) => { setCron({ hour: value }) }"
                    />
                    <span
                      class="text-center align-middle font-weight-bolder pt-1"
                      style="width: 0.5rem"
                    > : </span>
                    <b-form-input
                      id="cron-time-minute-input"
                      v-model="cronSchedule[0]"
                      type="number"
                      placeholder="mm"
                      min="0"
                      max="59"
                      style="max-width: 4.5rem"
                      :disabled="cron[0] === '*' || !cronInterval.length"
                      @change="(value) => { setCron({ min: value }) }"
                    />
                  </div>
                </b-form-group>
              </div>
              <div
                id="cron-date-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-group
                  label-cols-md="5"
                  label="Select Day of Month:"
                  class="m-0 pl-2"
                >
                  <div
                    class="d-flex justify-content-end"
                  >
                    <b-form-select
                      id="cron-time-date-input"
                      v-model="cronSchedule[2]"
                      :options="monthDays"
                      style="max-width: 9.5rem"
                      :disabled="cron[2] === '*' || !cronInterval.length"
                      @change="(value) => { setCron({ date: value }) }"
                    />
                  </div>
                </b-form-group>
              </div>
              <div
                id="cron-month-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-group
                  label-cols-md="4"
                  label="Select Month:"
                  class="m-0 pl-2"
                >
                  <div
                    class="d-flex justify-content-end"
                  >
                    <b-form-select
                      id="cron-time-month-input"
                      v-model="cronSchedule[3]"
                      :options="monthList"
                      style="max-width: 9.5rem"
                      :disabled="cron[3] === '*' || !cronInterval.length"
                      @change="(value) => { setCron({ month: value }) }"
                    />
                  </div>
                </b-form-group>
              </div>
              <div
                id="cron-weekday-container"
                class="px-2 pb-2 pt-0"
              >
                <b-form-group
                  label-cols-md="5"
                  label="Select Day of Week:"
                  class="m-0 pl-2"
                >
                  <div
                    class="d-flex justify-content-end"
                  >
                    <b-form-select
                      id="cron-day-input"
                      v-model="cronSchedule[4]"
                      style="max-width: 9.5rem"
                      :disabled="cron[4] === '*' || !cronInterval.length"
                      :options="weekDayOptions"
                      @change="(value) => { setCron({ day: value }) }"
                    />
                  </div>
                </b-form-group>
              </div>
              <div
                id="schedule-button-container"
                class="d-flex flex-row-reverse"
              >
                <b-button
                  variant="danger"
                  class="mr-2 my-1"
                  :disabled="!currentSchedule.id"
                  @click="deleteCurrentSchedule"
                >
                  Delete
                </b-button>
                <b-button
                  variant="danger"
                  class="mr-2 my-1"
                  @click="clearScheduleData"
                >
                  Clear
                </b-button>
                <b-button
                  variant="success"
                  class="mr-2 my-1"
                  :disabled="!isConnected"
                  @click="sendScheduleToRobot"
                >
                  Send
                </b-button>
                <b-button
                  variant="success"
                  class="mr-2 my-1"
                  :disabled="!isConnected || !currentPatrol.id
                    || !currentSchedule.obj.name || !isCronValid"
                  @click="saveSchedule"
                >
                  Save
                </b-button>
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
            :is-clickable="false"
            :show="isConnected"
            :zoom="mapZoom"
            :map-size="mapSize"
            :list="waypointList"
            :nb-of-waypoint="-1"
            :video-element="patrolHTMLElement"
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
      cron: ['', '', '', '', ''],
      cronInterval: '',
      isCronValid: '',
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
          text: 'Sunday',
          value: 'SUN',
        },
        {
          text: 'Monday',
          value: 'MON',
        },
        {
          text: 'Tuesday',
          value: 'TUE',
        },
        {
          text: 'Wednesday',
          value: 'WED',
        },
        {
          text: 'Thursday',
          value: 'THU',
        },
        {
          text: 'Friday',
          value: 'FRI',
        },
        {
          text: 'Saturday',
          value: 'SAT',
        },
      ],
      monthList: [
        {
          text: 'January',
          value: 1,
        },
        {
          text: 'February',
          value: 2,
        },
        {
          text: 'March',
          value: 3,
        },
        {
          text: 'April',
          value: 4,
        },
        {
          text: 'May',
          value: 5,
        },
        {
          text: 'June',
          value: 6,
        },
        {
          text: 'July',
          value: 7,
        },
        {
          text: 'August',
          value: 8,
        },
        {
          text: 'September',
          value: 9,
        },
        {
          text: 'October',
          value: 10,
        },
        {
          text: 'November',
          value: 11,
        },
        {
          text: 'December',
          value: 12,
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
    ...mapState({
      currentRobot: state => state.currentRobot,
      mapZoom: state => state.mapZoom,
      mapSize: state => state.mapSize,
      waypointList: state => state.patrol.current.obj.waypoints,
      patrolList: state => state.patrol.list,
      scheduleList: state => state.schedule.list,
      currentPatrol: state => state.patrol.current,
      currentSchedule: state => state.schedule.current,
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
    patrolSchedule() {
      const sc = [];
      this.scheduleList.forEach((s) => {
        if (s.info.patrolId === this.currentPatrol.id) {
          sc.push(s);
        }
      });
      return sc;
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
    selectedSchedule() {
      if (this.currentSchedule.id) {
        return {
          scheduleId: this.currentSchedule.id,
          patrolId: this.currentSchedule.obj.patrol,
          robotId: this.currentSchedule.obj.robot,
        };
      }
      return '';
    },
    weekDayValue() {
      const v = [];
      for (const day of this.weekDayOptions) {
        v.push(day.value);
      }
      return v;
    },
    cronSchedule() {
      const c = this.currentSchedule.obj.cron.split(' ');
      return [
        (c[0] === '*' ? '' : c[0]),
        (c[1] === '*' ? '' : c[1]),
        (c[2] === '*' ? '' : c[2]),
        (c[3] === '*' ? '' : c[3]),
        (c[4] === '*' ? '' : c[4]),
      ];
    },
    monthDays() {
      const l = [];
      for (let i = 1; i <= 31; i++) {
        l[i] = {
          text: i.toString(),
          value: i,
        };
      }
      return l;
    },
  },
  mounted() {
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    validateCron() {
      for (const unit of this.cron) {
        if (!unit) {
          this.isCronValid = false;
          return;
        }
      }
      this.isCronValid = true;
    },
    increaseZoom() {
      this.$store.commit('increaseMapZoom');
    },
    decreaseZoom() {
      this.$store.commit('decreaseMapZoom');
    },
    clearPatrolData() {
      this.$store.commit('clearCurrentPatrol');
    },
    clearScheduleData() {
      this.cron = ['', '', '', '', ''];
      this.cronInterval = [];
      this.$store.commit('clearCurrentSchedule');
    },
    deleteCurrentPatrol() {

    },
    deleteCurrentSchedule() {

    },
    setCron(event) {
      console.log(event);
      if (Array.isArray(event)) {
        this.cron = event;
      } else {
        const keys = Object.keys(event);
        if (keys.length === 1) {
          switch (keys[0]) {
            case 'min':
              this.cron[0] = event[keys[0]];
              break;
            case 'hour':
              this.cron[1] = event[keys[0]];
              break;
            case 'date':
              this.cron[2] = event[keys[0]];
              break;
            case 'month':
              this.cron[3] = event[keys[0]];
              break;
            case 'day':
              this.cron[4] = event[keys[0]];
              break;
            default:
              break;
          }
        }
      }
      this.validateCron();

      if (this.isCronValid) {
        this.$store.commit('setCurrentSchedule', { cron: this.cron.join(' ') });
      }
    },
    savePatrol() {
      if (!this.currentPatrol.obj.robot) {
        this.$store.commit('setCurrentPatrol', { robot: this.currentRobot.id.db });
      }
      this.$store.commit('setCurrentPatrol', { last_modified: new Date().toISOString() });
      this.$store.dispatch('database/savePatrol', this.currentPatrol)
        .then(() => {
          this.$store.dispatch('database/queryPatrols')
            .catch((err) => {
              console.log(err);
            });
        }).catch((err) => {
          console.log(err);
        });
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
    saveSchedule() {
      if (!this.currentSchedule.obj.robot) {
        this.$store.commit('setCurrentSchedule', { robot: this.currentRobot.id.db });
      }
      if (!this.currentSchedule.obj.patrol) {
        this.$store.commit('setCurrentSchedule', { patrol: this.currentPatrol.id });
      }
      this.$store.commit('setCurrentSchedule', { last_modified: new Date().toISOString() });
      this.$store.commit('setCurrentSchedule', { enabled: true });
      this.$store.dispatch('database/saveSchedule', this.currentSchedule)
        .then(() => {
          this.$store.dispatch('database/querySchedules')
            .catch((err) => {
              console.log(err);
            });
        }).catch((err) => {
          console.log(err);
        });
    },
    loadSchedule(event) {
      if (event) {
        this.$store.dispatch('database/getSchedule', event);
      } else {
        this.$store.commit('setCurrentScheduleId', '');
        this.$store.commit('clearCurrentSchedule');
      }
    },
    sendPatrolToRobot() {
      this.$store.dispatch('sendPatrol', { patrol: this.currentPatrol.obj });
    },
    sendScheduleToRobot() {
      this.$store.dispatch('sendPatrol', { patrol: this.currentPatrol.obj, repetitions: this.currentSchedule.obj.repetitions });
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
.overlay-container {
  top: 5px;
  right: 15px;
  z-index: 100;
  max-width: 80px;
  height: auto;
}
</style>
