<template>
  <div class="w-100 h-100 border rounded table-container">
    <div class="mh-100 overflow-auto rounded">
      <!-- Table -->
      <table
        id="waypoint-table"
        class="h-100 table table-borderless table-striped table-hover border-left border-right m-0"
        style="text-align: center"
      >
        <!-- Header -->
        <thead class="text-white">
          <th
            class="position-sticky"
            :style="{ width: columnWidth }"
          >
            #
          </th>
          <th
            v-for="header in headers"
            :key="header"
            class="position-sticky"
            :style="{ width: columnWidth }"
          >
            {{ header }}
          </th>
          <th
            v-if="removable"
            class="position-sticky"
            :style="{ width: columnWidth }"
          >
            Remove
          </th>
        </thead>
        <!-- Body -->
        <tbody>
          <!-- Row Template -->
          <template v-for="(el, index) of list">
            <tr
              :key="index"
            >
              <!-- # -->
              <td :style="{ width: columnWidth }">
                {{ (index+1).toFixed(0) }}
              </td>
              <!-- Dynamic -->
              <td
                v-for="(value, key) in el"
                :key="key"
                :style="{ width: columnWidth }"
              >
                <div v-if="typeof value === 'number'">
                  {{ value.toFixed(1) }}
                </div>
                <div v-else>
                  {{ value }}
                </div>
              </td>
              <!-- Remove button column -->
              <td
                v-if="removable"
                :style="{ width: columnWidth }"
              >
                <!-- Remove button -->
                <button
                  :id="'removeBtn'+index"
                  type="button"
                  class="btn btn-danger p-0 m-0 border border-secondary h-100 w-50"
                  @click="removeIndex(index)"
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
 * A generic vue table component that allows for dynamic columns and offers a remove row button as
 * its last row that emits an event when clicked.
 *
 * NOTE: In the future, the remove button and its event might be changed to become more generic and
 * make configurable (style).
 * @displayName Table
 * @since 0.2.2
 * @version 1.0.0
 */
export default {
  name: 'securbot-table',
  props: {
    /**
     * The desired headers.
     */
    headers: {
      type: Array,
      required: true,
    },
    /**
     * The list of object to enumerate by row.
     */
    list: {
      type: Array,
      required: true,
    },
    /**
     * Flag for the remove button column.
     */
    removable: {
      type: Boolean,
      default: false,
    },
  },
  data() {
    return {
      validProp: true,
      columnWidth: '20%',
    };
  },
  mounted() {
    if (this.list.length !== this.headers.length) {
      this.validProp = false;
    }
    const nbCol = this.headers.length + (this.removable ? 2 : 1);
    this.columnWidth = `${(100 / nbCol)}%`;
  },
  methods: {
    /**
     * Gets called when a remove button is clicked.
     * @param {Number} index The row index of the clicked button.
     * @public
     */
    removeIndex(index) {
      console.log(index);
      /**
       * The remove row event.
       *
       * @type {Number} The index of the row to remove.
       */
      this.$emit('removeRow', index);
    },
  },
};
</script>

<style scoped>
table {
  position: relative;
}
th {
  background-color: #00A759;
  position: sticky;
  top: 0;
}
.table-container {
  background-color: rgb(255, 255, 255);
  box-shadow: 0 2px 2px 0 rgba(0, 0, 0, 0.5), 0 3px 3px 0 rgba(0, 54, 5, 0.19);
}
</style>
