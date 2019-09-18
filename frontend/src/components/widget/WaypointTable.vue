<template>
  <!-- Waypoint Table widget -->
  <div class="w-100 h-100">
    <!-- Overflow container -->
    <div class="mh-100 overflow-auto">
      <!-- Table -->
      <table
        id="waypoint-table"
        class="table tabl e-borderless table-striped border-left border-right"
        style="text-align: center"
      >
        <!-- Header -->
        <thead class="text-white bg-green-sb">
          <th style="width:20%">
            #
          </th>
          <th style="width:20%">
            X
          </th>
          <th style="width:20%">
            Y
          </th>
          <th style="width:20%">
            Yaw
          </th>
          <th style="width:20%">
            Remove
          </th>
        </thead>
        <!-- Body -->
        <tbody>
          <!-- Table row per waypoint in list -->
          <template v-for="(waypoint, index) of waypointList">
            <tr
              :key="waypoint.dateTime"
              class="border-bottom"
            >
              <td style="width:20%">
                {{ (index+1).toFixed(0) }}
              </td>
              <td style="width:20%">
                {{ waypoint.x.toFixed(1) }}
              </td>
              <td style="width:20%">
                {{ waypoint.y.toFixed(1) }}
              </td>
              <td style="width:20%">
                {{ waypoint.yaw.toFixed(1) }}
              </td>
              <td style="width:20%">
                <!-- Remove button -->
                <button
                  :id="'removeBtn'+index"
                  type="button"
                  class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                  @click="removeWaypointFromList(index)"
                >
                  <img
                    src="~/open-iconic/svg/trash.svg"
                    alt=""
                    style="width:12px;height:12px"
                  >
                </button>
              </td>
            </tr>
          </template>
        </tbody>
      </table>
    </div>
  </div>
</template>

<script>
/**
 * Vue SFC used as a widget to show waypoints in a array given as props.
 * The waypoints are shown in a html table generated with vue v-for.
 * The table contains a colomn of buttons use to delete the row they
 * are in and, at the same time, delete the corresponding waypoint in
 * the array.
 * This component have the following dependency :
 * Bootstrap-Vue for styling.
 *
 *
 * @module widget/WaypointTable
 * @vue-prop {Object[]} waypointList - Lists the current waypoints
 */

/* Disabled comment documentation
 * Might use those eventually by forking jsdoc-vue-js so it can manage the author
 * and version tag correctly
 * @author Valerie Gauthier <valerie.gauthier@usherbrooke.ca>
 * @author Edouard Legare <edouard.legare@usherbrooke.ca>
 * @version 1.0.0
 */

export default {
  name: 'waypoint-table',
  props: {
    waypointList: {
      type: Object,
      required: true,
    },
  },
  data() {
    return {
    };
  },
  methods: {
    /**
     * Used to clear the patrol or empty the waypoint list.
     * @method
     * @param {Number} index - Index of the waypoint to remove from list.
     */
    removeWaypointFromList(index) {
      this.waypointList.splice(index, 1);
    },
  },
};
</script>

<style>
.bg-trash{
  display: block;
  background: url("~/open-iconic/svg/trash.svg");
  fill: black;
}
</style>
