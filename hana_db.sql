--
-- PostgreSQL database dump
--

-- Dumped from database version 16.9 (Ubuntu 16.9-0ubuntu0.24.04.1)
-- Dumped by pg_dump version 16.9 (Ubuntu 16.9-0ubuntu0.24.04.1)

SET statement_timeout = 0;
SET lock_timeout = 0;
SET idle_in_transaction_session_timeout = 0;
SET client_encoding = 'UTF8';
SET standard_conforming_strings = on;
SELECT pg_catalog.set_config('search_path', '', false);
SET check_function_bodies = false;
SET xmloption = content;
SET client_min_messages = warning;
SET row_security = off;

--
-- Name: pgcrypto; Type: EXTENSION; Schema: -; Owner: -
--

CREATE EXTENSION IF NOT EXISTS pgcrypto WITH SCHEMA public;


--
-- Name: EXTENSION pgcrypto; Type: COMMENT; Schema: -; Owner: 
--

COMMENT ON EXTENSION pgcrypto IS 'cryptographic functions';


SET default_tablespace = '';

SET default_table_access_method = heap;

--
-- Name: hana_bots; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.hana_bots (
    hana_bot_id integer NOT NULL,
    bot_name character varying(50) NOT NULL,
    battery integer,
    status character varying,
    CONSTRAINT hana_bots_status_check CHECK (((status)::text = ANY ((ARRAY['충전중'::character varying, '작업중'::character varying, '대기중'::character varying, '복귀중'::character varying])::text[])))
);


ALTER TABLE public.hana_bots OWNER TO postgres;

--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.hana_bots_hana_bot_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.hana_bots_hana_bot_id_seq OWNER TO postgres;

--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.hana_bots_hana_bot_id_seq OWNED BY public.hana_bots.hana_bot_id;


--
-- Name: items; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.items (
    item_id integer NOT NULL,
    item_type character varying NOT NULL,
    item_quantity integer DEFAULT 100 NOT NULL,
    CONSTRAINT items_item_type_check CHECK (((item_type)::text = ANY ((ARRAY['물'::character varying, '영양제'::character varying, '생필품'::character varying, '식판'::character varying])::text[])))
);


ALTER TABLE public.items OWNER TO postgres;

--
-- Name: items_item_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.items_item_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.items_item_id_seq OWNER TO postgres;

--
-- Name: items_item_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.items_item_id_seq OWNED BY public.items.item_id;


--
-- Name: locations; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.locations (
    location_id integer NOT NULL,
    location_type character varying NOT NULL,
    location_name character varying(100) NOT NULL,
    x_coord double precision,
    y_coord double precision,
    yaw double precision NOT NULL,
    CONSTRAINT locations_location_type_check CHECK (((location_type)::text = ANY ((ARRAY['서비스 스테이션'::character varying, '픽업 스테이션'::character varying, '충전 스테이션'::character varying, '선반'::character varying])::text[])))
);


ALTER TABLE public.locations OWNER TO postgres;

--
-- Name: locations_location_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.locations_location_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.locations_location_id_seq OWNER TO postgres;

--
-- Name: locations_location_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.locations_location_id_seq OWNED BY public.locations.location_id;


--
-- Name: login_logs; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.login_logs (
    id integer NOT NULL,
    resident_id integer,
    success boolean,
    client_ip character varying(50),
    login_time timestamp without time zone DEFAULT CURRENT_TIMESTAMP
);


ALTER TABLE public.login_logs OWNER TO postgres;

--
-- Name: login_logs_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.login_logs_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.login_logs_id_seq OWNER TO postgres;

--
-- Name: login_logs_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.login_logs_id_seq OWNED BY public.login_logs.id;


--
-- Name: reserved_tasks; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.reserved_tasks (
    reserved_task_id integer NOT NULL,
    task_type character varying NOT NULL,
    requester_resident_id integer,
    item_id integer,
    target_location integer,
    scheduled_time timestamp without time zone NOT NULL,
    CONSTRAINT reserved_tasks_task_type_check CHECK (((task_type)::text = ANY ((ARRAY['식사 배달'::character varying, '물품 배달'::character varying])::text[])))
);


ALTER TABLE public.reserved_tasks OWNER TO postgres;

--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.reserved_tasks_reserved_task_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.reserved_tasks_reserved_task_id_seq OWNER TO postgres;

--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.reserved_tasks_reserved_task_id_seq OWNED BY public.reserved_tasks.reserved_task_id;


--
-- Name: residents; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.residents (
    resident_id integer NOT NULL,
    name character varying(50) NOT NULL,
    gender character(1) NOT NULL,
    birth_date date NOT NULL,
    assigned_room_id integer,
    service_station_id integer,
    login_id character varying(50) NOT NULL,
    password character varying(255) DEFAULT public.crypt('1234'::text, public.gen_salt('bf'::text)) NOT NULL,
    CONSTRAINT residents_gender_check CHECK ((gender = ANY (ARRAY['M'::bpchar, 'F'::bpchar])))
);


ALTER TABLE public.residents OWNER TO postgres;

--
-- Name: residents_resident_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.residents_resident_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.residents_resident_id_seq OWNER TO postgres;

--
-- Name: residents_resident_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.residents_resident_id_seq OWNED BY public.residents.resident_id;


--
-- Name: tasks; Type: TABLE; Schema: public; Owner: postgres
--

CREATE TABLE public.tasks (
    task_id integer NOT NULL,
    task_type character varying NOT NULL,
    status character varying NOT NULL,
    requester_resident_id integer,
    item_id integer,
    target_location_id integer,
    source_reserved_task_id integer,
    assigned_bot_id integer,
    created_at timestamp without time zone DEFAULT CURRENT_TIMESTAMP NOT NULL,
    completed_at timestamp without time zone,
    CONSTRAINT tasks_status_check CHECK (((status)::text = ANY ((ARRAY['대기'::character varying, '할당'::character varying, '집기중'::character varying, '이동중'::character varying, '수령대기'::character varying, '완료'::character varying, '실패'::character varying])::text[]))),
    CONSTRAINT tasks_task_type_check CHECK (((task_type)::text = ANY ((ARRAY['배달'::character varying, '호출'::character varying])::text[])))
);


ALTER TABLE public.tasks OWNER TO postgres;

--
-- Name: tasks_task_id_seq; Type: SEQUENCE; Schema: public; Owner: postgres
--

CREATE SEQUENCE public.tasks_task_id_seq
    AS integer
    START WITH 1
    INCREMENT BY 1
    NO MINVALUE
    NO MAXVALUE
    CACHE 1;


ALTER SEQUENCE public.tasks_task_id_seq OWNER TO postgres;

--
-- Name: tasks_task_id_seq; Type: SEQUENCE OWNED BY; Schema: public; Owner: postgres
--

ALTER SEQUENCE public.tasks_task_id_seq OWNED BY public.tasks.task_id;


--
-- Name: hana_bots hana_bot_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots ALTER COLUMN hana_bot_id SET DEFAULT nextval('public.hana_bots_hana_bot_id_seq'::regclass);


--
-- Name: items item_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.items ALTER COLUMN item_id SET DEFAULT nextval('public.items_item_id_seq'::regclass);


--
-- Name: locations location_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations ALTER COLUMN location_id SET DEFAULT nextval('public.locations_location_id_seq'::regclass);


--
-- Name: login_logs id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs ALTER COLUMN id SET DEFAULT nextval('public.login_logs_id_seq'::regclass);


--
-- Name: reserved_tasks reserved_task_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks ALTER COLUMN reserved_task_id SET DEFAULT nextval('public.reserved_tasks_reserved_task_id_seq'::regclass);


--
-- Name: residents resident_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents ALTER COLUMN resident_id SET DEFAULT nextval('public.residents_resident_id_seq'::regclass);


--
-- Name: tasks task_id; Type: DEFAULT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks ALTER COLUMN task_id SET DEFAULT nextval('public.tasks_task_id_seq'::regclass);


--
-- Data for Name: hana_bots; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.hana_bots (hana_bot_id, bot_name, battery, status) FROM stdin;
7	HANABOT_7	\N	\N
8	HANABOT_8	\N	\N
9	HANABOT_9	\N	\N
10	HANA_PINKY	100	대기중
\.


--
-- Data for Name: items; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.items (item_id, item_type, item_quantity) FROM stdin;
10	영양제	82
11	생필품	87
9	물	74
12	식판	94
\.


--
-- Data for Name: locations; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.locations (location_id, location_type, location_name, x_coord, y_coord, yaw) FROM stdin;
1	서비스 스테이션	ROOM1	0.1	0.78	-0.707
3	충전 스테이션	CHARGER1	0	0	1
5	픽업 스테이션	PICKUP	0.33	-0.33	-1
\.


--
-- Data for Name: login_logs; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.login_logs (id, resident_id, success, client_ip, login_time) FROM stdin;
1	1	t	127.0.0.1	2025-07-26 21:50:28.65401
2	\N	f	127.0.0.1	2025-07-26 22:07:01.334553
3	\N	f	127.0.0.1	2025-07-26 22:07:45.598618
4	\N	f	127.0.0.1	2025-07-26 22:07:48.718039
5	\N	f	127.0.0.1	2025-07-26 22:07:58.408657
6	2	t	127.0.0.1	2025-07-26 23:19:43.445885
7	2	t	127.0.0.1	2025-07-26 23:20:55.638729
8	1	t	127.0.0.1	2025-07-26 23:21:07.190418
10	1	t	127.0.0.1	2025-07-26 23:40:34.070484
11	\N	f	127.0.0.1	2025-07-26 23:40:43.927193
12	9999	t	127.0.0.1	2025-07-26 23:43:19.206874
13	9999	t	127.0.0.1	2025-07-26 23:44:17.40516
14	9999	t	127.0.0.1	2025-07-26 23:44:22.980135
18	9999	t	127.0.0.1	2025-07-26 23:47:42.697985
19	9999	t	127.0.0.1	2025-07-27 04:35:31.416605
20	9999	t	127.0.0.1	2025-07-27 04:51:35.367336
21	9999	t	127.0.0.1	2025-07-27 05:00:14.938507
22	9999	t	124.49.97.226	2025-07-27 05:10:39.667467
23	\N	f	192.168.219.115	2025-07-27 05:40:41.777295
24	9999	t	192.168.219.115	2025-07-27 05:40:44.891617
25	\N	f	192.168.219.111	2025-07-27 05:46:40.218719
26	9999	t	192.168.219.111	2025-07-27 05:46:51.442256
27	9999	t	127.0.0.1	2025-07-27 05:54:37.151836
28	9999	t	127.0.0.1	2025-07-27 06:00:23.820719
29	9999	t	127.0.0.1	2025-07-27 06:00:24.688395
30	9999	t	192.168.219.115	2025-07-27 08:39:34.504958
31	9999	t	192.168.219.115	2025-07-27 08:46:31.070564
32	9999	t	192.168.219.115	2025-07-27 11:02:48.812189
33	9999	t	192.168.219.115	2025-07-27 11:04:42.199085
34	9999	t	192.168.219.115	2025-07-27 11:06:15.105766
35	9999	t	127.0.0.1	2025-07-27 11:43:56.361823
36	2	t	127.0.0.1	2025-07-27 12:47:00.915778
37	\N	f	192.168.219.111	2025-07-27 12:55:29.295987
38	9999	t	192.168.219.111	2025-07-27 12:55:36.648054
39	\N	f	192.168.0.6	2025-07-28 01:24:26.248823
40	9999	t	192.168.0.6	2025-07-28 01:24:34.860888
41	2	t	192.168.0.19	2025-07-28 01:25:43.725823
42	9999	t	192.168.0.6	2025-07-28 03:18:16.262451
43	9999	t	192.168.0.7	2025-07-28 09:21:03.931387
44	\N	f	127.0.0.1	2025-07-28 09:24:02.546585
45	9999	t	127.0.0.1	2025-07-28 09:24:05.070081
46	9999	t	192.168.0.13	2025-07-28 09:25:27.511021
47	\N	f	192.168.0.13	2025-07-28 09:27:12.886656
48	\N	f	192.168.0.13	2025-07-28 09:27:17.758605
49	\N	f	192.168.0.13	2025-07-28 09:27:19.903993
50	\N	f	192.168.0.13	2025-07-28 09:27:21.318129
51	9999	t	192.168.0.13	2025-07-28 09:27:26.904852
52	9999	t	192.168.0.7	2025-07-29 10:28:11.807015
53	\N	f	192.168.0.7	2025-07-29 10:30:34.040631
54	9999	t	192.168.0.7	2025-07-29 10:37:17.7109
55	9999	t	192.168.0.7	2025-07-29 10:44:26.966008
56	9999	t	192.168.0.7	2025-07-29 10:46:00.208971
57	9999	t	192.168.0.7	2025-07-29 10:46:22.197675
58	9999	t	127.0.0.1	2025-07-29 12:49:10.356638
59	9999	t	127.0.0.1	2025-07-29 14:14:03.488422
60	9999	t	192.168.219.115	2025-07-29 14:28:45.141284
61	9999	t	192.168.219.115	2025-07-29 15:07:39.406453
62	9999	t	192.168.219.115	2025-07-29 15:31:43.067347
63	9999	t	192.168.219.111	2025-07-29 15:57:28.204878
64	9999	t	192.168.219.111	2025-07-29 16:19:57.260177
\.


--
-- Data for Name: reserved_tasks; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.reserved_tasks (reserved_task_id, task_type, requester_resident_id, item_id, target_location, scheduled_time) FROM stdin;
\.


--
-- Data for Name: residents; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.residents (resident_id, name, gender, birth_date, assigned_room_id, service_station_id, login_id, password) FROM stdin;
2	김순자	F	1930-01-01	\N	\N	soon_30	$2a$06$wExDrXZHKyrQR90HXYyTSOqNGJeIO8KhfJLwVQwK0foOWf5QJgmhS
1	김복자	F	1940-01-01	\N	\N	40bokja	$2a$06$PTSh8ggRGKVxT1p5d0ZDl.glBkgOaRN7RWb3hX8L5pfMz8D7BR29O
9999	admin	M	1900-01-01	\N	\N	admin	$2a$06$BOFmpxmSteP7o5KQ2gfG4u/1hcr0m8xECPm7GQtQXW9GB/JwL13vi
\.


--
-- Data for Name: tasks; Type: TABLE DATA; Schema: public; Owner: postgres
--

COPY public.tasks (task_id, task_type, status, requester_resident_id, item_id, target_location_id, source_reserved_task_id, assigned_bot_id, created_at, completed_at) FROM stdin;
9	배달	완료	9999	9	\N	\N	10	2025-07-29 15:57:41.475438	2025-07-29 15:58:42.471243
11	배달	완료	9999	9	\N	\N	10	2025-07-29 16:03:56.247605	2025-07-29 16:04:40.397819
4	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:41:15.405506	2025-07-29 15:42:50.362658
16	배달	실패	9999	9	\N	\N	10	2025-07-29 17:13:17.0771	2025-07-29 17:18:42.864109
5	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:41:22.686985	2025-07-29 15:42:54.274096
13	배달	완료	9999	10	\N	\N	10	2025-07-29 16:10:29.076258	2025-07-29 16:12:51.029105
8	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:50:40.818904	2025-07-29 15:52:20.031877
3	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:35:37.88762	2025-07-29 15:40:30.241157
10	배달	완료	9999	10	\N	\N	10	2025-07-29 15:58:02.176751	2025-07-29 15:59:26.922245
14	배달	완료	9999	9	\N	\N	10	2025-07-29 16:10:37.827045	2025-07-29 16:15:21.677167
15	배달	완료	9999	9	\N	\N	10	2025-07-29 16:20:06.351885	2025-07-29 16:22:23.21619
12	배달	실패	9999	10	\N	\N	10	2025-07-29 16:04:02.150444	2025-07-29 16:39:47.664703
18	배달	실패	9999	9	\N	\N	10	2025-07-29 17:27:08.167814	2025-07-29 17:28:25.53631
19	배달	실패	9999	9	\N	\N	\N	2025-07-29 17:29:09.029702	2025-07-29 17:30:19.686309
7	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:43:57.005422	2025-07-29 15:46:50.012951
21	배달	완료	9999	9	\N	\N	10	2025-07-29 17:49:11.355123	2025-07-29 17:49:59.927517
6	배달	완료	9999	\N	\N	\N	10	2025-07-29 15:43:47.224697	2025-07-29 15:44:51.196828
20	배달	완료	9999	9	\N	\N	10	2025-07-29 17:38:19.429736	2025-07-29 17:39:42.038697
17	배달	실패	9999	11	\N	\N	10	2025-07-29 17:13:47.123178	2025-07-29 17:22:57.570537
\.


--
-- Name: hana_bots_hana_bot_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.hana_bots_hana_bot_id_seq', 10, true);


--
-- Name: items_item_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.items_item_id_seq', 13, true);


--
-- Name: locations_location_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.locations_location_id_seq', 1, false);


--
-- Name: login_logs_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.login_logs_id_seq', 64, true);


--
-- Name: reserved_tasks_reserved_task_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.reserved_tasks_reserved_task_id_seq', 1, false);


--
-- Name: residents_resident_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.residents_resident_id_seq', 2, true);


--
-- Name: tasks_task_id_seq; Type: SEQUENCE SET; Schema: public; Owner: postgres
--

SELECT pg_catalog.setval('public.tasks_task_id_seq', 21, true);


--
-- Name: hana_bots hana_bots_bot_name_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots
    ADD CONSTRAINT hana_bots_bot_name_key UNIQUE (bot_name);


--
-- Name: hana_bots hana_bots_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.hana_bots
    ADD CONSTRAINT hana_bots_pkey PRIMARY KEY (hana_bot_id);


--
-- Name: items items_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.items
    ADD CONSTRAINT items_pkey PRIMARY KEY (item_id);


--
-- Name: locations locations_location_name_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations
    ADD CONSTRAINT locations_location_name_key UNIQUE (location_name);


--
-- Name: locations locations_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.locations
    ADD CONSTRAINT locations_pkey PRIMARY KEY (location_id);


--
-- Name: login_logs login_logs_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT login_logs_pkey PRIMARY KEY (id);


--
-- Name: reserved_tasks reserved_tasks_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_pkey PRIMARY KEY (reserved_task_id);


--
-- Name: residents residents_login_id_key; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents
    ADD CONSTRAINT residents_login_id_key UNIQUE (login_id);


--
-- Name: residents residents_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.residents
    ADD CONSTRAINT residents_pkey PRIMARY KEY (resident_id);


--
-- Name: tasks tasks_pkey; Type: CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_pkey PRIMARY KEY (task_id);


--
-- Name: idx_rtasks_sched_time; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_rtasks_sched_time ON public.reserved_tasks USING btree (scheduled_time);


--
-- Name: idx_tasks_created_at; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_tasks_created_at ON public.tasks USING btree (created_at);


--
-- Name: idx_tasks_status; Type: INDEX; Schema: public; Owner: postgres
--

CREATE INDEX idx_tasks_status ON public.tasks USING btree (status);


--
-- Name: login_logs fk_loginlogs_resident; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.login_logs
    ADD CONSTRAINT fk_loginlogs_resident FOREIGN KEY (resident_id) REFERENCES public.residents(resident_id);


--
-- Name: reserved_tasks reserved_tasks_item_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_item_id_fkey FOREIGN KEY (item_id) REFERENCES public.items(item_id);


--
-- Name: reserved_tasks reserved_tasks_requester_resident_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_requester_resident_id_fkey FOREIGN KEY (requester_resident_id) REFERENCES public.residents(resident_id);


--
-- Name: reserved_tasks reserved_tasks_target_location_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.reserved_tasks
    ADD CONSTRAINT reserved_tasks_target_location_fkey FOREIGN KEY (target_location) REFERENCES public.locations(location_id);


--
-- Name: tasks tasks_assigned_bot_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_assigned_bot_id_fkey FOREIGN KEY (assigned_bot_id) REFERENCES public.hana_bots(hana_bot_id);


--
-- Name: tasks tasks_item_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_item_id_fkey FOREIGN KEY (item_id) REFERENCES public.items(item_id);


--
-- Name: tasks tasks_requester_resident_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_requester_resident_id_fkey FOREIGN KEY (requester_resident_id) REFERENCES public.residents(resident_id);


--
-- Name: tasks tasks_source_reserved_task_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_source_reserved_task_id_fkey FOREIGN KEY (source_reserved_task_id) REFERENCES public.reserved_tasks(reserved_task_id);


--
-- Name: tasks tasks_target_location_id_fkey; Type: FK CONSTRAINT; Schema: public; Owner: postgres
--

ALTER TABLE ONLY public.tasks
    ADD CONSTRAINT tasks_target_location_id_fkey FOREIGN KEY (target_location_id) REFERENCES public.locations(location_id);


--
-- PostgreSQL database dump complete
--

