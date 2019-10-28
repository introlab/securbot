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
            <u>Filters:</u>
          </h4>
          <div
            id="inner-filter-container"
            class="border rounded mx-1 my-0 p-2 overflow-auto"
            style="height: calc( 100% - 60px - 50px);"
          >
            <b-container fluid>
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
          class="h-100 w-100"
        >
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
          <securbot-table
            v-else
            style="max-height: 100%"
            :headers="headers"
            :list="list"
          />
        </div>
      </b-col>
    </b-row>
  </b-jumbotron>
</template>

<script>
import { mapState, mapGetters } from 'vuex';
import SecurbotTable from '../generic/Table';

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
    };
  },
  computed: {
    ...mapState('database', {
      headers: state => state.headers,
      list: state => state.eventList,
      robots: state => state.robots,
      tagList: state => state.tagList,
      querying: state => state.queryingDB,
      display: (state) => {
        // eslint-disable-next-line no-underscore-dangle
        const _display = [];
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
  },
  methods: {
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
      const { filters } = this.filters;
      this.$store.dispatch('filterEvents', filters);
    },
  },
};
</script>

<style>
</style>
