
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


function toTagList(parameter, {req, loc, path}) {
    if (typeof parameter === 'string')
    {
        try {
            parameter = JSON.parse(parameter)
        }
        catch(e) {}
    }

    if (typeof parameter === 'string')
        parameter = [parameter]

    return parameter
}

function checkTagList(parameter, {req, loc, path}) {
    try {
        return parameter.every((tag) => typeof tag === 'string')
    }
    catch(e)
    {
        throw new Error('Invalid tag list')
    }
}



module.exports = {
    tag_and: {
        optional: true,
        customSanitizer: { options: toTagList },
        custom: { options: checkTagList },
    },
    tag_or: {
        optional: true,
        customSanitizer: { options: toTagList },
        custom: { options: checkTagList },
    },
    tag_not: {
        optional: true,
        customSanitizer: { options: toTagList },
        custom: { options: checkTagList },
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

    // Minimized list contains fewer fields
    minimized: {
        optional: true,
        isBoolean: true,
        toBoolean: true,
        errorMessage: '`minimized` muste be `true` or `false`'
    }
}
