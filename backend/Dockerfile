FROM node

ENV PORT 8080

WORKDIR /opt/web_rtc

COPY package*.json ./

RUN npm install

COPY ./server.js ./server.js

EXPOSE $PORT

CMD ["npm", "start"]
