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
            <waypoint-table />
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
/**
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */
import { mapState } from 'vuex';
import PatrolMap from '../widgets/PatrolMap';
import WaypointTable from '../widgets/WaypointTable';
import SaveLoad from '../widgets/SaveLoad';

export default {
  name: 'patrol',
  components: {
    PatrolMap,
    WaypointTable,
    SaveLoad,
  },
  computed: mapState({
    waypointList: state => state.patrol.waypointList,
    patrolList: state => state.patrol.patrolList,
  }),
  /**
   * Lifecycle Hook - mounted
   *
   * @method
   * @listens mount(el)
   */
  mounted() {
    this.$store.dispatch('getPatrols');
    this.$store.dispatch('updateHTMLVideoElements');
  },
  methods: {
    /**
     * Callback used to get patrols on DB
     * @method
     */
    getSavedPatrols() {
    },
    /**
     * Callback used to send the patrol to the connected robot robot
     * @method
     */
    sendPatrol() {
      console.log('Sendig patrolPlan:');
      if (this.waypointList.length) {
        this.$store.dispatch('sendPatrol', this.waypointList);
      }
    },
    savePatrols() {
      this.$store.dispatch('savePatrols', this.patrolList);
    },
    /**
     * Callback used to clear the patrol
     * @method
     */
    clearWaypointList() {
      this.$store.commit('clearWaypointList');
    },
    /**
     * Method used to clear the patrol list (delete the list on db)
     * @method
     */
    clearPatrolList() {
      this.$store.commit('clearPatrol');
    },
  },
};
</script>

<style>

</style>
