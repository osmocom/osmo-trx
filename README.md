About OsmoTRX
=============

OsmoTRX is a software-defined radio transceiver that implements the Layer 1
physical layer of a BTS comprising the following 3GPP specifications:

* TS 05.01 *Physical layer on the radio path*
* TS 05.02 *Multiplexing and Multiple Access on the Radio Path*
* TS 05.04 *Modulation*
* TS 05.10 *Radio subsystem synchronization*

OsmoTRX is originally based on the transceiver code from the
[OpenBTS](https://osmocom.org/projects/osmobts/wiki/OpenBTS) project, but setup
to operate independently with the purpose of using with non-OpenBTS software and
projects, specifically within the Osmocom stack.  Used together with
[OsmoBTS](https://osmocom.org/projects/osmobts/wiki) you can get a pretty
standard GSM/GPRS/EGPRS BTS with Abis interface as per the relevant 3GPP specifications.

Homepage
--------

The official homepage of the project is
<https://osmocom.org/projects/osmotrx/wiki/OsmoTRX>

GIT Repository
--------------

You can clone from the official osmo-trx.git repository using

        git clone https://gitea.osmocom.org/cellular-infrastructure/osmo-trx`

There is a web interface at <https://gitea.osmocom.org/cellular-infrastructure/osmo-trx>

Documentation
-------------

Doxygen-generated API documentation is generated during the build process, but
also available online for each of the sub-libraries at User Manual for OsmoTRX
can be generated during the build process, and is also available online at
<https://ftp.osmocom.org/docs/latest/osmotrx-usermanual.pdf>.

Mailing List
------------

Discussions related to OsmoTRX are happening on the openbsc@lists.osmocom.org
mailing list, please see <https://lists.osmocom.org/mailman/listinfo/openbsc>
for subscription options and the list archive.

Please observe the [Osmocom Mailing List
Rules](https://osmocom.org/projects/cellular-infrastructure/wiki/Mailing_List_Rules)
when posting.

Contributing
------------

Our coding standards are described at
<https://osmocom.org/projects/cellular-infrastructure/wiki/Coding_standards>

We us a gerrit based patch submission/review process for managing contributions.
Please see <https://osmocom.org/projects/cellular-infrastructure/wiki/Gerrit>
for more details

The current patch queue for OsmoTRX can be seen at
<https://gerrit.osmocom.org/q/project:osmo-trx+status:open>
