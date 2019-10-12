<template>
  <!-- SaveLoad widget -->
  <div class="h-100 w-100 d-flex flex-column">
    <!-- Header container -->
    <div
      class="btn-toolbar mb-1 w-100 d-flex flex-row-reverse"
      style="height:40px;"
      role="toolbar"
    >
      <!-- Save patrol button -->
      <button
        type="button"
        class="btn btn-success h-100"
        style="align-items: center; width:125px; min-width:125px;"
        @click="addPatrolToPatrolList()"
      >
        Save patrol
      </button>
      <!-- Patrol name input -->
      <div
        class="h-100 w-auto text-left mr-1"
        style="flex:1;"
      >
        <input
          id="nameTextBox"
          v-model="newPatrolName"
          type="text"
          :placeholder="[[ placehold ]]"
          class="form-control h-100"
        >
      </div>
    </div>
    <!-- Patrol List container -->
    <div
      class="w-100"
      style="height: calc(100% - 40px - 0.25rem)"
    >
      <!-- Overflow container -->
      <div class="mh-100 overflow-auto">
        <!-- Patrol table -->
        <table
          id="saved-patrol-table"
          class="table table-borderless table-striped border-right border-left"
          style="text-align: center"
        >
          <!-- Header -->
          <thead
            class="text-white bg-green-sb"
          >
            <th
              class="w-50"
              style="text-align:left"
            >
              Patrols
            </th>
            <th class="w-25">
              Choose
            </th>
            <th class="w-25">
              Remove
            </th>
          </thead>
          <!-- Table body -->
          <tbody>
            <!-- Table row per patrol in list -->
            <tr
              v-for="(patrol,index) of patrolList"
              :key="patrol.Name"
              class="border-bottom"
            >
              <!-- Patrol Name -->
              <td
                class="w-50"
                style="text-align:left"
              >
                {{ patrol.Name }}
              </td>
              <!-- Select button -->
              <td class="w-25">
                <button
                  :id="'selectBtn'+index"
                  type="button"
                  class="btn btn-success p-0 m-0 border border-secondary h-100 w-50"
                  @click="selectPatrolFromList(index)"
                >
                  <img
                    src="~/open-iconic/svg/check.svg"
                    alt=""
                    style="width:12px;height:12px;"
                  >
                </button>
              </td>
              <!-- Remove button -->
              <td class="w-25">
                <button
                  :id="'removeBtn'+index"
                  type="button"
                  class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                  @click="removePatrolFromList(index)"
                >
                  <img
                    src="~/open-iconic/svg/trash.svg"
                    alt=""
                    style="width:12px;height:12px;"
                  >
                </button>
              </td>
            </tr>
          </tbody>
        </table>
      </div>
    </div>
  </div>
</template>

<script>
import { mapState } from 'vuex';

/**
 * A component that dynamically generates a table with given array. Allows to select and remove
 * any element from the list. This component will soon be deprecated.
 *
 * Authors:
 *
 *    - Valerie Gauthier - <valerie.gauthier@usherbrooke.ca>
 *    - Edouard Legare - <edouard.legare@usherbrooke.ca>
 * @since 0.1.0
 * @version 0.2.0
 * @displayName Patrol List
 */
export default {
  name: 'save-load',
  data() {
    return {
      newPatrolName: '',
      placehold: 'Patrol name',
    };
  },
  computed: mapState({
    waypointList: state => state.patrol.waypointList,
    patrolList: state => state.patrol.patrolList,
  }),
  methods: {
    /**
     * Removes an element from the list.
     *
     * @param {Number} index The element index to remove.
     * @public
     */
    removePatrolFromList(index) {
      this.$store.commit('removePatrol', index);
    },
    /**
     * Adds an element to the list.
     *
     * @public
     */
    addPatrolToPatrolList() {
      const patrol = [];
      if (this.newPatrolName === '') {
        this.placehold = 'Enter name';
      } else {
        patrol.Name = this.newPatrolName;
        patrol.waypoints = Array.from(this.waypointList);
        this.$store.commit('addPatrol', patrol);
        this.newPatrolName = '';
        this.placehold = 'Patrol name   ';
        console.log(this.patrolList);
      }
    },
    /**
     * Select an element from the list.
     *
     * @param {Number} index The element index to select.
     * @public
     */
    selectPatrolFromList(index) {
      if (this.waypointList.length) {
        console.log('overwrite plan');
        this.$store.commit('clearWaypointList');
      } else {
        console.log(this.patrolList[index].waypoints);
      }
      console.log(this.patrolList[index]);
      this.patrolList[index].waypoints.forEach((element) => {
        this.$store.commit('addWaypoint', { wp: element });
      });
    },
  },
};
</script>

<style>
</style>
