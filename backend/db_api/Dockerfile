FROM node

ENV PORT 8080

COPY package*.json main.js ./
COPY api ./api

RUN npm install --production

# MongoDB waiter
ADD https://github.com/ufoscout/docker-compose-wait/releases/download/2.2.1/wait /wait
RUN chmod +x /wait

CMD /wait && npm start
