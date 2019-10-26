
// Query schema
// every filter is optional
// {
//     tag_and: [String],
//     tag_or: [String],
//     tag_not: [String],
//     search_expression: String,
//     alert: Boolean,
//     viewed: Boolean,
//     before: Date, (ISO 8601)
//     after: Date (ISO 8601)
// }
module.exports = {
    tag_and: {
        optional: true,
        toArray: true,
        custom: {
            options: (array, {req, loc, path}) => {
                return array.every((tag) => typeof tag === 'string')
            }
        },
        errorMessage: 'Invalid tag_whitelist syntax'
    },
    tag_or: {
        optional: true,
        toArray: true,
        custom: {
            options: (array, {req, loc, path}) => {
                return array.every((tag) => typeof tag === 'string')
            }
        },
        errorMessage: 'Invalid `tag_whitelist` syntax'
    },
    tag_not: {
        optional: true,
        toArray: true,
        custom: {
            options: (array, {req, loc, path}) => {
                return array.every((tag) => typeof tag === 'string')
            }
        },
        errorMessage: 'Invalid `tag_whitelist` syntax'
    },
    search_expression: {
        optional: true,
        isString: true,
        errorMessage: '`search_expression` must be a String'
    },
    alert: {
        optional: true,
        isBoolean: true,
        toBoolean: true,
        errorMessage: '`alert` must be `true` or `false`'
    },
    viewed: {
        optional: true,
        isBoolean: true,
        toBoolean: true,
        errorMessage: '`viewed` must be `true` or `false`'
    },
    before: {
        optional: true,
        isISO8601: true,
        toDate: true,
        errorMessage: '`before` must be a valid ISO 8601 date'
    },
    after: {
        optional: true,
        isISO8601: true,
        toDate: true,
        errorMessage: '`after` must be a valid ISO 8601 date'
    },
}
