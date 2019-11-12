<template>
  <!-- Event page -->
  <b-jumbotron
    id="event-layout"
    :fluid="true"
    :container-fluid="true"
    class="h-100 w-100 bg-transparent"
  >
    <b-row class="h-100">
      <b-col
        id="filter-col"
        md="3"
        class="mh-100"
      >
        <div
          id="filter-container"
          class="h-100 w-100 border rounded shadow-sb"
          style="max-height: 100%"
        >
          <h4 class="m-3">
            Filters
          </h4>
          <div
            id="inner-filter-container"
            class="border rounded mx-1 my-0 p-2 overflow-auto"
            style="height: calc( 100% - 60px - 50px);"
          >
            <b-container fluid>
              <b-row>
                <b-form-select
                  v-model="selectedFilter"
                  :options="predefFilters"
                  text-field="name"
                  value-field="filters"
                  class="m-2"
                  @change="setLocalFilters"
                >
                  <template v-slot:first>
                    <option value="" disabled>-- Predefined filters... --</option>
                  </template>
                </b-form-select>
              </b-row>
              <b-row>
                <b-col
                  sm="4"
                >
                  <label
                    for="robot-select-filter"
                    class="mt-2"
                  >
                    Robot:
                  </label>
                </b-col>
                <b-col
                  sm="8"
                >
                  <b-form-select
                    id="robot-select-filter"
                    v-model="filters.robot"
                    class="ml-2"
                    :options="display"
                  />
                </b-col>
              </b-row>
              <b-row>
                <b-col
                  sm="4"
                >
                  <label
                    for="after-input-filter"
                    class="mt-2"
                  >
                    After:
                  </label>
                </b-col>
                <b-col
                  sm="8"
                >
                  <b-form-input
                    id="after-input-filter"
                    v-model="filters.afterDate"
                    class="ml-2"
                    type="date"
                  />
                </b-col>
              </b-row>
              <b-row>
                <b-col
                  sm="4"
                >
                  <label
                    for="before-input-filter"
                    class="mt-2"
                  >
                    Before:
                  </label>
                </b-col>
                <b-col
                  sm="8"
                >
                  <b-form-input
                    id="before-input-filter"
                    v-model="filters.beforeDate"
                    class="ml-2"
                    type="date"
                  />
                </b-col>
              </b-row>
              <b-row class="mt-2">
                <h6
                  class="w-100"
                  style="margin-left: 15px;"
                >
                  Other filters:
                </h6>
                <b-col>
                  <b-form-checkbox
                    id="new-filter-checkbox"
                    v-model="filters.other.onlyNew"
                    :value="true"
                    :unchecked-value="false"
                  >
                    Only New
                  </b-form-checkbox>
                </b-col>
                <b-col>
                  <b-form-checkbox
                    id="alert-filter-checkbox"
                    v-model="filters.other.notify"
                    :value="true"
                    :unchecked-value="false"
                  >
                    Alerts
                  </b-form-checkbox>
                </b-col>
              </b-row>
              <!-- Tag Filters -->
              <b-row class="mt-3 position-relative">
                <b-badge
                  class="position-absolute"
                  style="z-index: 100; top: -5px; left: 10px;"
                  variant="primary"
                >
                  Include
                </b-badge>
                <b-col sm="12">
                  <div
                    id="include-tag-filter"
                    class="border rounded p-3 d-flex flex-row flex-wrap"
                    style="min-height: 80px;"
                  >
                    <button
                      v-for="tag in tagList"
                      :key="tag"
                      class="m-1"
                      :class="[ isIncludeTagSelected(tag)
                        ? 'btn btn-primary'
                        : 'btn btn-secondary' ]"
                      :disabled="isExcludeTagSelected(tag)"
                      @click="tagIncludeClicked(tag)"
                    >
                      {{ tag }}
                    </button>
                  </div>
                </b-col>
              </b-row>
              <b-row class="mt-3 position-relative">
                <b-badge
                  class="position-absolute"
                  style="z-index: 100; top: -5px; left: 10px;"
                  variant="danger"
                >
                  Exclude
                </b-badge>
                <b-col sm="12">
                  <div
                    id="exclude-tag-filter p-3"
                    class="border rounded p-3"
                    style="min-height: 80px;"
                  >
                    <button
                      v-for="tag in tagList"
                      :key="tag"
                      class="m-1"
                      :class="[ isExcludeTagSelected(tag)
                        ? 'btn btn-danger'
                        : 'btn btn-secondary' ]"
                      :disabled="isIncludeTagSelected(tag)"
                      @click="tagExcludeClicked(tag)"
                    >
                      {{ tag }}
                    </button>
                  </div>
                </b-col>
              </b-row>
              <b-row class="my-3 position-relative">
                <b-col>
                  <div id="search-for-filter">
                    <b-form-textarea
                      id="search-for-input-filter"
                      v-model="filters.textSearch"
                      placeholder="Search for..."
                      rows="3"
                      max-rows="6"
                    />
                  </div>
                </b-col>
              </b-row>
            </b-container>
          </div>
          <div
            style="height: 50px;"
          >
            <div class="h-100 w-100 d-flex flex-row-reverse align-items-center">
              <b-button
                variant="primary"
                class="mr-2"
                style="max-height: 35px"
                :disabled="querying"
                @click="applyFilter"
              >
                Apply Filters
              </b-button>
            </div>
          </div>
        </div>
      </b-col>
      <b-col
        id="table-col"
        md="9"
        class="mh-100"
      >
        <div
          id="table-container"
          class="h-100 w-100 position-relative"
        >
          <div
            v-if="isConnected && viewMap"
            class="position-absolute overlay-container"
          >
            <div
              id="event-overlay-button-container"
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
          <div
            class="position-absolute"
            style="top:-35px;right:10px;z-index:10;"
          >
            <toggle-button
              :value="viewMap"
              :color="switchColor"
              :sync="true"
              :labels="{checked: 'map', unchecked: 'list'}"
              :disabled="!isConnected"
              @change="changeMapView"
            />
          </div>
          <div
            v-if="querying"
            id="spinner-container"
            style="background-color: gray; opacity: 0.5;"
            class="border rounded h-100 w-100 d-flex align-items-center justify-content-center"
          >
            <b-spinner
              variant="primary"
              type="grow"
              label="Spinning"
            />
          </div>
          <div
            v-else-if="queryError"
            id="error-container"
            style="background-color: crimson; opacity: 0.25;"
            class="border rounded h-100 w-100 d-flex align-items-center justify-content-center"
          >
            <h3
              style="color: crimson"
            >
              Something wrong happen...
            </h3>
          </div>
          <securbot-table
            v-else-if="!viewMap"
            style="max-height: 100%"
            :headers="headers"
            :list="list"
          />
          <div
            v-else-if="viewMap && isConnected"
            class="h-100 w-100 m-auto position-relative"
          >
            <video-box
              :show="true"
              :zoom="mapZoom"
              :video-id="eventId"
            />
            <waypoint-overlay
              :is-active="true"
              :is-clickable="false"
              :list="eventsWaypoints"
              :zoom="mapZoom"
              :map-size="mapSize"
              :nb-of-waypoint="eventsWaypoints.length"
              :video-element="eventElement"
              :refresh-rate="1"
            />
          </div>
          <div
            v-else
          />
        </div>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>

<script>
import { mapState, mapGetters } from 'vuex';
import { ToggleButton } from 'vue-js-toggle-button';
import SecurbotTable from '../generic/Table';
import VideoBox from '../widgets/VideoBox';
import WaypointOverlay from '../generic/WaypointOverlay';

/**
 * This page will be used to show events from a database. It will allow the operator to filter
 * those elements and should refresh when new data where sent to the database.
 *
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @since 0.1.0
 * @displayName Events and Logging View
 * @version 0.1.0
 */
export default {
  name: 'event-page',
  components: {
    SecurbotTable,
    VideoBox,
    WaypointOverlay,
    ToggleButton,
  },
  data() {
    return {
      filters: {
        robot: 'all',
        beforeDate: '',
        afterDate: '',
        other: {
          onlyNew: false,
          notify: false,
        },
        includeTags: [],
        excludeTags: [],
        textSearch: '',
      },
      selectedFilter: '',
      viewMap: false,
      switchColor: {
        checked: '#00A759',
        unchecked: '#00A759',
        disabled: '#E8E8E8',
      },
    };
  },
  computed: {
    ...mapGetters('database', [
      'eventsWaypoints',
    ]),
    ...mapState({
      mapZoom: state => state.mapZoom,
      mapSize: state => state.mapSize,
      eventId: state => state.htmlElement.eventId,
      eventElement: state => state.htmlElement.event,
      isConnected: state => state.client.connectionState.robot === 'connected',
    }),
    ...mapState('database', {
      headers: state => state.headers,
      predefFilters: state => JSON.parse(JSON.stringify(state.predefFilters)),
      list: state => state.events,
      robots: state => state.robots,
      tagList: state => state.tagList,
      querying: state => state.queryingDB,
      queryError: state => state.errorDuringQuery,
      display: (state) => {
        // eslint-disable-next-line no-underscore-dangle
        const _display = [{ text: 'all', value: 'all' }];
        state.robots.forEach((robot) => {
          _display.push({
            text: robot.name,
            value: robot.id,
          });
        });
        return _display;
      },
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
    setLocalFilters(event) {
      const f = {
        includeTags: (event.tag_and ? event.tag_and : []),
        excludeTags: (event.tag_not ? event.tag_not : []),
        textSearch: (event.search_expression ? event.search_expression : ''),
        other: {
          onlyNew: event.viewed ? true : '',
          notify: event.alert ? true : '',
        },
        beforeDate: (event.before ? event.before.slice(0, 10) : ''),
        afterDate: (event.after ? event.after.slice(0, 10) : ''),
      };
      Object.assign(this.filters, f);
    },
    changeMapView(event) {
      this.viewMap = event.value;
      this.$nextTick(() => {
        this.$store.dispatch('updateHTMLVideoElements');
      });
    },
    isIncludeTagSelected(tag) {
      return this.filters.includeTags.includes(tag);
    },
    isExcludeTagSelected(tag) {
      return this.filters.excludeTags.includes(tag);
    },
    tagIncludeClicked(tag) {
      console.log(tag);
      console.log(this.isIncludeTagSelected(tag));
      if (this.isIncludeTagSelected(tag)) {
        this.filters.includeTags.splice(this.filters.includeTags.indexOf(tag), 1);
      } else {
        this.filters.includeTags.push(tag);
      }
    },
    tagExcludeClicked(tag) {
      if (this.isExcludeTagSelected(tag)) {
        this.filters.excludeTags.splice(this.filters.excludeTags.indexOf(tag), 1);
      } else {
        this.filters.excludeTags.push(tag);
      }
    },
    applyFilter() {
      this.viewMap = false;
      const { filters } = this;
      this.$store.commit('database/resetQuery');
      this.$store.commit('database/resetEvents');
      this.$store.commit('database/setRobotFilters', filters);
      this.$store.commit('database/setEventFilters', filters);
      this.$store.dispatch('database/filterEvents');
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
  right: 0px;
  z-index: 100;
  max-width: 80px;
  height: auto;
}
</style>
