FROM node

ENV PORT 8080

COPY package*.json main.js ./

RUN npm install --production

CMD ["node", "."]
